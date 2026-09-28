use std::ffi::CStr;
use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream, ToSocketAddrs};
use std::os::raw::{c_char, c_int, c_uchar, c_uint, c_void};
use std::os::unix::io::FromRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::{Duration, Instant};
use tungstenite::{accept, Message};

const INDEX_HTML: &str = include_str!("web/index.html");

// FTDI C FFI Bindings
#[repr(C)]
struct FtdiContext {
    _private: [u8; 0],
}

const BITMODE_RESET: c_uchar = 0x00;
const BITMODE_SYNCFF: c_uchar = 0x40;
const SIGINT: c_int = 2;
const SIGTERM: c_int = 15;

#[link(name = "ftdi1")]
extern "C" {
    fn ftdi_new() -> *mut FtdiContext;
    fn ftdi_free(ftdi: *mut FtdiContext);
    fn ftdi_set_interface(ftdi: *mut FtdiContext, interface: c_int) -> c_int;
    fn ftdi_usb_open(ftdi: *mut FtdiContext, vendor: c_int, product: c_int) -> c_int;
    fn ftdi_usb_close(ftdi: *mut FtdiContext) -> c_int;
    fn ftdi_set_bitmode(ftdi: *mut FtdiContext, bitmask: c_uchar, mode: c_uchar) -> c_int;
    fn ftdi_set_latency_timer(ftdi: *mut FtdiContext, latency: c_uchar) -> c_int;
    fn ftdi_read_data_set_chunksize(ftdi: *mut FtdiContext, chunksize: c_uint) -> c_int;
    fn ftdi_read_data(ftdi: *mut FtdiContext, buf: *mut c_uchar, size: c_int) -> c_int;
    fn ftdi_get_error_string(ftdi: *mut FtdiContext) -> *const c_char;
}

static RUNNING: AtomicBool = AtomicBool::new(true);

extern "C" fn sig_handler(_sig: c_int) {
    RUNNING.store(false, Ordering::SeqCst);
}

#[derive(Clone)]
struct SpectrumFrame {
    peak_freq_mhz: f32, // Sub-bin interpolated frequency
    peak_val: u16,
    peak_bin: u16,
    fps: u16,
    mb_s: f32,
    live_bins: [u16; 512],     // Live or video-averaged bins
    max_hold_bins: [u16; 512], // Max-hold peak retention bins
}

impl Default for SpectrumFrame {
    fn default() -> Self {
        Self {
            peak_freq_mhz: 0.0,
            peak_val: 0,
            peak_bin: 0,
            fps: 0,
            mb_s: 0.0,
            live_bins: [0u16; 512],
            max_hold_bins: [0u16; 512],
        }
    }
}

// User-controllable DSP parameters
struct DspConfig {
    avg_alpha: f32,   // Video averaging smoothing factor (0.01..1.0)
    reset_max: bool,  // Reset max-hold request
}

impl Default for DspConfig {
    fn default() -> Self {
        Self {
            avg_alpha: 0.25,
            reset_max: false,
        }
    }
}

// PrefixedStream allows WebSocket handshake inspection on the same port as HTTP
struct PrefixedStream {
    prefix: Vec<u8>,
    pos: usize,
    stream: TcpStream,
}

impl Read for PrefixedStream {
    fn read(&mut self, buf: &mut [u8]) -> std::io::Result<usize> {
        if self.pos < self.prefix.len() {
            let n = (self.prefix.len() - self.pos).min(buf.len());
            buf[..n].copy_from_slice(&self.prefix[self.pos..self.pos + n]);
            self.pos += n;
            Ok(n)
        } else {
            self.stream.read(buf)
        }
    }
}

impl Write for PrefixedStream {
    fn write(&mut self, buf: &[u8]) -> std::io::Result<usize> {
        self.stream.write(buf)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.stream.flush()
    }
}

fn bind_reuse(addr: &str) -> std::io::Result<TcpListener> {
    let sock_addr = addr
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::Other, "Invalid addr"))?;

    let fd = unsafe { libc::socket(libc::AF_INET, libc::SOCK_STREAM, 0) };
    if fd < 0 {
        return Err(std::io::Error::last_os_error());
    }

    let optval: libc::c_int = 1;
    unsafe {
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_REUSEADDR,
            &optval as *const _ as *const _,
            std::mem::size_of_val(&optval) as _,
        );
        libc::setsockopt(
            fd,
            libc::SOL_SOCKET,
            libc::SO_REUSEPORT,
            &optval as *const _ as *const _,
            std::mem::size_of_val(&optval) as _,
        );
    }

    let c_addr = match sock_addr {
        std::net::SocketAddr::V4(v4) => {
            let mut sin: libc::sockaddr_in = unsafe { std::mem::zeroed() };
            sin.sin_family = libc::AF_INET as libc::sa_family_t;
            sin.sin_port = v4.port().to_be();
            sin.sin_addr.s_addr = u32::from_ne_bytes(v4.ip().octets());
            sin
        }
        _ => {
            unsafe { libc::close(fd) };
            return Err(std::io::Error::new(
                std::io::ErrorKind::Other,
                "Only IPv4 supported",
            ));
        }
    };

    let res = unsafe {
        libc::bind(
            fd,
            &c_addr as *const _ as *const _,
            std::mem::size_of_val(&c_addr) as _,
        )
    };
    if res < 0 {
        unsafe { libc::close(fd) };
        return Err(std::io::Error::last_os_error());
    }

    let res = unsafe { libc::listen(fd, 128) };
    if res < 0 {
        unsafe { libc::close(fd) };
        return Err(std::io::Error::last_os_error());
    }

    Ok(unsafe { TcpListener::from_raw_fd(fd) })
}

fn main() {
    println!("================================================================================");
    println!(" Cyclone IV Real-Time FFT Spectrum Server (50 MSPS / FT232H Sync FIFO)         ");
    println!(" Features: Sub-Bin Parabolic Interpolation + Video Averaging + Max Hold        ");
    println!("================================================================================");
    println!("Web Client: http://localhost:8080");
    println!("================================================================================");

    unsafe {
        libc::signal(libc::SIGINT, sig_handler as usize);
        libc::signal(libc::SIGTERM, sig_handler as usize);
        libc::signal(libc::SIGPIPE, libc::SIG_IGN);
    }

    let latest_frame = Arc::new(Mutex::new(SpectrumFrame::default()));
    let dsp_config = Arc::new(RwLock::new(DspConfig::default()));
    let ws_senders: Arc<Mutex<Vec<Sender<Vec<u8>>>>> = Arc::new(Mutex::new(Vec::new()));

    // -------------------------------------------------------------------------
    // 1. FT232H Acquisition & DSP Thread
    // -------------------------------------------------------------------------
    let frame_ftdi = latest_frame.clone();
    let dsp_config_ftdi = dsp_config.clone();

    let ftdi_thread = thread::spawn(move || {
        let ftdi = unsafe { ftdi_new() };
        if ftdi.is_null() {
            eprintln!("Error: ftdi_new failed");
            return;
        }

        unsafe {
            ftdi_set_interface(ftdi, 0);

            if ftdi_usb_open(ftdi, 0x0403, 0x6014) < 0 {
                let err = CStr::from_ptr(ftdi_get_error_string(ftdi)).to_string_lossy();
                eprintln!("[FTDI] Error opening FT232H (0403:6014): {}", err);
                ftdi_free(ftdi);
                return;
            }

            ftdi_set_bitmode(ftdi, 0x00, BITMODE_RESET);
            thread::sleep(Duration::from_millis(10));
            ftdi_set_bitmode(ftdi, 0xFF, BITMODE_SYNCFF);
            ftdi_set_latency_timer(ftdi, 2);
            ftdi_read_data_set_chunksize(ftdi, 256 * 1024);
        }

        println!("[FTDI] Connected to FT232H in Synchronous FIFO mode.");

        const CHUNK_SIZE: usize = 256 * 1024;
        let mut raw_buf = vec![0u8; CHUNK_SIZE];
        let mut stream_buf = Vec::with_capacity(CHUNK_SIZE * 2);

        let mut current_frame: Vec<u16> = Vec::with_capacity(1024);
        let mut in_frame = false;
        let mut frame_valid = true;
        // Tracks which word indices received a synthetic zero due to a USB slip.
        // These bins are skipped in the EMA update to preserve their last valid value.
        let mut zero_filled_bins = [false; 1024];

        // DSP State: Video Averaging & Max-Hold
        let mut video_avg_bins = [0.0f32; 512];
        let mut max_hold_bins = [0u16; 512];
        let mut initialized_avg = false;

        let mut last_stat_time = Instant::now();
        let mut last_log_time = Instant::now();
        let mut frames_count: u32 = 0;
        let mut bytes_count: u64 = 0;
        let mut current_fps: u16 = 0;
        let mut current_mb_s: f32 = 0.0;

        let mut stat_total_frames: u64 = 0;
        let mut stat_exact_1024: u64 = 0;
        let mut stat_1020_1023: u64 = 0;
        let mut stat_under_1020: u64 = 0;

        const BIN_WIDTH_HZ: f32 = 50_000_000.0 / 1024.0; // 48828.125 Hz

        while RUNNING.load(Ordering::Relaxed) {
            let n = unsafe { ftdi_read_data(ftdi, raw_buf.as_mut_ptr(), CHUNK_SIZE as c_int) };
            if n <= 0 {
                thread::sleep(Duration::from_millis(1));
                continue;
            }

            let n = n as usize;
            bytes_count += n as u64;
            stream_buf.extend_from_slice(&raw_buf[..n]);

            // Periodic Throughput & Rate calculation
            let now = Instant::now();
            let elapsed = now.duration_since(last_stat_time);
            if elapsed >= Duration::from_millis(200) {
                let dt = elapsed.as_secs_f32();
                current_fps = (frames_count as f32 / dt).round() as u16;
                current_mb_s = (bytes_count as f32 / (1024.0 * 1024.0)) / dt;
                frames_count = 0;
                bytes_count = 0;
                last_stat_time = now;
            }

            let elapsed_log = now.duration_since(last_log_time);
            if elapsed_log >= Duration::from_secs(1) {
                if stat_total_frames > 0 {
                    let pct_exact = (stat_exact_1024 as f64 / stat_total_frames as f64) * 100.0;
                    let pct_1020_1024 = ((stat_exact_1024 + stat_1020_1023) as f64 / stat_total_frames as f64) * 100.0;
                    println!(
                        "[Frame Stats] Total: {} | Exact 1024: {} ({:.1}%) | 1020..1024: {} ({:.1}%) | <1020: {}",
                        stat_total_frames, stat_exact_1024, pct_exact,
                        stat_exact_1024 + stat_1020_1023, pct_1020_1024, stat_under_1020
                    );
                    stat_total_frames = 0;
                    stat_exact_1024 = 0;
                    stat_1020_1023 = 0;
                    stat_under_1020 = 0;
                }
                last_log_time = now;
            }

            // Check client configuration updates
            let (alpha, reset_max) = {
                let mut cfg = dsp_config_ftdi.write().unwrap();
                let r = cfg.reset_max;
                cfg.reset_max = false;
                (cfg.avg_alpha, r)
            };

            if reset_max {
                max_hold_bins = [0u16; 512];
            }

            // Word Stream Parser
            let mut i = 0;
            let len = stream_buf.len();

            while i < len - 1 {
                let b0 = stream_buf[i];
                let b1 = stream_buf[i + 1];
                let tag = b1 & 0xF0;

                if tag == 0xB0 {
                    // SOP: Start of new 1024-Point FFT Frame
                    if in_frame && frame_valid {
                        let flen = current_frame.len();
                        stat_total_frames += 1;
                        if flen == 1024 {
                            stat_exact_1024 += 1;
                        } else if flen >= 1020 && flen < 1024 {
                            stat_1020_1023 += 1;
                        } else {
                            stat_under_1020 += 1;
                        }
                    }

                    if in_frame && frame_valid && current_frame.len() >= 1020 {
                        frames_count += 1;

                        // Decode signed 6-bit source exponent embedded in SOP word (Bin 0)
                        let raw_exp = (current_frame[0] & 0x3F) as u8;
                        let exp_signed: i8 = if (raw_exp & 0x20) != 0 {
                            (raw_exp | 0xC0) as i8
                        } else {
                            raw_exp as i8
                        };

                        // Reference exponent: -9 is nominal for 1024-point FFT with full-scale square wave.
                        // Each shift difference scales power by 2^(2 * exp_diff) = 4^(exp_diff).
                        const EXP_REF: i8 = -9;
                        let exp_diff = EXP_REF - exp_signed;
                        let power_scale = (2.0f32).powi((exp_diff * 2) as i32);

                        // Apply source_exp normalization to positive bins 1..512
                        let mut norm_bins = [0u16; 512];
                        norm_bins[0] = 0; // DC suppressed
                        let available = current_frame.len().min(512);
                        for k in 1..available {
                            let raw_v = current_frame[k] as f32;
                            norm_bins[k] = (raw_v * power_scale).clamp(0.0, 4095.0) as u16;
                        }

                        let frame_512 = &norm_bins[..512];

                        // 1. Update Video Exponential Moving Average & Max-Hold
                        // Bins with a USB slip zero-fill are skipped to preserve the last
                        // valid average value instead of pulling toward 0.
                        if !initialized_avg {
                            for k in 0..512 {
                                if !zero_filled_bins[k] {
                                    video_avg_bins[k] = frame_512[k] as f32;
                                    max_hold_bins[k] = frame_512[k];
                                }
                            }
                            initialized_avg = true;
                        } else {
                            for k in 0..512 {
                                if zero_filled_bins[k] {
                                    continue; // Hold last valid average, don't update with synthetic 0
                                }
                                let val = frame_512[k];
                                video_avg_bins[k] = (1.0 - alpha) * video_avg_bins[k] + alpha * (val as f32);
                                if val > max_hold_bins[k] {
                                    max_hold_bins[k] = val;
                                }
                            }
                        }

                        // 2. Coarse Peak Search in positive frequency bins 1..510
                        let mut peak_val: u16 = 0;
                        let mut peak_bin: usize = 1;
                        for k in 1..511 {
                            let v = video_avg_bins[k].round() as u16;
                            if v > peak_val {
                                peak_val = v;
                                peak_bin = k;
                            }
                        }

                        // 3. Sub-Bin Parabolic Peak & Power Interpolation
                        let (peak_freq_mhz, true_peak_val) = if peak_bin > 1 && peak_bin < 510 && peak_val > 20 {
                            let alpha_val = video_avg_bins[peak_bin - 1];
                            let beta_val = video_avg_bins[peak_bin];
                            let gamma_val = video_avg_bins[peak_bin + 1];

                            let denom = alpha_val - 2.0 * beta_val + gamma_val;
                            let p = if denom.abs() > 1e-4 {
                                (0.5 * (alpha_val - gamma_val) / denom).clamp(-0.5, 0.5)
                            } else {
                                0.0
                            };

                            let true_bin = (peak_bin as f32) + p;
                            let freq = (true_bin * BIN_WIDTH_HZ) / 1_000_000.0;
                            // Continuous true peak power at exact frequency vertex
                            let continuous_power = (beta_val - 0.25 * (alpha_val - gamma_val) * p).round().clamp(0.0, 4095.0) as u16;
                            (freq, continuous_power)
                        } else {
                            ((peak_bin as f32 * BIN_WIDTH_HZ) / 1_000_000.0, peak_val)
                        };

                        // Convert averaged bins to u16
                        let mut live_bins = [0u16; 512];
                        for k in 0..512 {
                            live_bins[k] = video_avg_bins[k].round().clamp(0.0, 4095.0) as u16;
                        }

                        // Publish to shared state
                        if let Ok(mut lock) = frame_ftdi.try_lock() {
                            lock.peak_freq_mhz = peak_freq_mhz;
                            lock.peak_val = true_peak_val;
                            lock.peak_bin = peak_bin as u16;
                            lock.fps = current_fps;
                            lock.mb_s = current_mb_s;
                            lock.live_bins = live_bins;
                            lock.max_hold_bins = max_hold_bins;
                        }
                    }

                    // Start new frame: verify exponent (-9 is 0x37 in lower 6 bits)
                    // and verify next word has tag 0xA0 if available
                    let next_word_valid = if i + 3 < len {
                        (stream_buf[i + 3] & 0xF0) == 0xA0
                    } else {
                        true
                    };

                    if (b0 & 0x3F) == 0x37 && next_word_valid {
                        let val = (((b1 & 0x0F) as u16) << 8) | (b0 as u16);
                        current_frame.clear();
                        current_frame.push(val);
                        zero_filled_bins = [false; 1024];
                        in_frame = true;
                        frame_valid = true;
                        i += 2;
                    } else {
                        // Spurious SOP or unaligned byte boundary slip
                        in_frame = false;
                        frame_valid = false;
                        current_frame.clear();
                        i += 1;
                    }
                } else if tag == 0xA0 {
                    if in_frame {
                        let val = (((b1 & 0x0F) as u16) << 8) | (b0 as u16);
                        if current_frame.len() < 1024 {
                            current_frame.push(val);
                        }
                    }
                    i += 2;
                } else {
                    // 510-byte USB micro-packet boundary slip detected.
                    // The FTDI strips 2 modem-status bytes per 512-byte USB packet, delivering
                    // 510 bytes (255 words) per micro-packet. Since 255 is odd, every packet
                    // boundary shifts the byte phase by 1, causing a slip once every 255 words.
                    //
                    // FIX: Insert a synthetic zero word at the slip position to keep all
                    // subsequent bins at their correct word indices. Without this, every bin
                    // after the slip would be shifted by -1, corrupting harmonic peaks.
                    // The zero_filled_bins bitmask ensures this bin is SKIPPED in the EMA
                    // update, preserving the last valid average instead of dipping toward 0.
                    if in_frame && current_frame.len() < 1024 {
                        let slip_idx = current_frame.len();
                        current_frame.push(0u16);
                        zero_filled_bins[slip_idx] = true;
                    }
                    i += 1;
                }
            }

            stream_buf.drain(..i);
        }

        println!("[FTDI] Closing USB interface...");
        unsafe {
            ftdi_set_bitmode(ftdi, 0xFF, BITMODE_RESET);
            ftdi_usb_close(ftdi);
            ftdi_free(ftdi);
        }
    });

    // -------------------------------------------------------------------------
    // 2. Broadcast Thread (60 FPS Binary Spectrum Packets)
    // -------------------------------------------------------------------------
    let frame_bcast = latest_frame.clone();
    let ws_senders_bcast = ws_senders.clone();

    let broadcast_thread = thread::spawn(move || {
        while RUNNING.load(Ordering::Relaxed) {
            let start = Instant::now();

            let has_clients = {
                let senders = ws_senders_bcast.lock().unwrap();
                !senders.is_empty()
            };

            if has_clients {
                let frame = {
                    let lock = frame_bcast.lock().unwrap();
                    lock.clone()
                };

                // Binary packet layout (2062 bytes):
                // 0..4:     f32 peak_freq_mhz (interpolated, LE)
                // 4..6:     u16 peak_val (LE)
                // 6..8:     u16 peak_bin (LE)
                // 8..10:    u16 fps (LE)
                // 10..14:   f32 mb_s (LE)
                // 14..1038: [u16; 512] live_bins (LE)
                // 1038..2062: [u16; 512] max_hold_bins (LE)
                let mut packet = Vec::with_capacity(2062);
                packet.extend_from_slice(&frame.peak_freq_mhz.to_le_bytes());
                packet.extend_from_slice(&frame.peak_val.to_le_bytes());
                packet.extend_from_slice(&frame.peak_bin.to_le_bytes());
                packet.extend_from_slice(&frame.fps.to_le_bytes());
                packet.extend_from_slice(&frame.mb_s.to_le_bytes());
                for &b in &frame.live_bins {
                    packet.extend_from_slice(&b.to_le_bytes());
                }
                for &b in &frame.max_hold_bins {
                    packet.extend_from_slice(&b.to_le_bytes());
                }

                let mut senders = ws_senders_bcast.lock().unwrap();
                senders.retain(|tx| tx.send(packet.clone()).is_ok());
            }

            // Target 60 FPS (~16.6 ms)
            let elapsed = start.elapsed();
            if elapsed < Duration::from_millis(16) {
                thread::sleep(Duration::from_millis(16) - elapsed);
            }
        }
    });

    // -------------------------------------------------------------------------
    // 3. HTTP + WebSocket Server (Listening on 0.0.0.0:8080)
    // -------------------------------------------------------------------------
    let listener = bind_reuse("0.0.0.0:8080").expect("Failed to bind on 0.0.0.0:8080");
    println!("[Server] HTTP and WebSocket server running at http://0.0.0.0:8080\n");

    for stream in listener.incoming() {
        if !RUNNING.load(Ordering::Relaxed) {
            break;
        }

        let mut stream = match stream {
            Ok(s) => s,
            Err(_) => continue,
        };

        let ws_senders_client = ws_senders.clone();
        let dsp_config_client = dsp_config.clone();

        thread::spawn(move || {
            let mut initial_buf = [0u8; 4096];
            let n = match stream.read(&mut initial_buf) {
                Ok(n) if n > 0 => n,
                _ => return,
            };

            let req_str = String::from_utf8_lossy(&initial_buf[..n]);
            let is_websocket = req_str.to_lowercase().contains("upgrade: websocket");

            if !is_websocket {
                // Serve HTML Web Client
                if req_str.starts_with("GET /") || req_str.starts_with("HEAD /") {
                    let is_head = req_str.starts_with("HEAD /");
                    let body = if is_head { "" } else { INDEX_HTML };
                    let resp = format!(
                        "HTTP/1.1 200 OK\r\nContent-Type: text/html; charset=utf-8\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                        INDEX_HTML.len(),
                        body
                    );
                    let _ = stream.write_all(resp.as_bytes());
                } else {
                    let resp = "HTTP/1.1 404 NOT FOUND\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
                    let _ = stream.write_all(resp.as_bytes());
                }
                return;
            }

            // WebSocket Handshake
            let prefixed = PrefixedStream {
                prefix: initial_buf[..n].to_vec(),
                pos: 0,
                stream,
            };

            let mut websocket = match accept(prefixed) {
                Ok(ws) => ws,
                Err(e) => {
                    eprintln!("[WS] Handshake failed: {:?}", e);
                    return;
                }
            };

            println!("[WS] Client connected.");
            let (tx, rx): (Sender<Vec<u8>>, Receiver<Vec<u8>>) = channel();
            {
                let mut senders = ws_senders_client.lock().unwrap();
                senders.push(tx);
            }

            if let Err(e) = websocket.get_mut().stream.set_nonblocking(true) {
                eprintln!("[WS] set_nonblocking error: {:?}", e);
            }

            while RUNNING.load(Ordering::Relaxed) {
                while let Ok(frame_data) = rx.try_recv() {
                    if let Err(e) = websocket.send(Message::Binary(frame_data.into())) {
                        if !matches!(e, tungstenite::Error::Io(ref io_err) if io_err.kind() == std::io::ErrorKind::WouldBlock) {
                            println!("[WS] Client disconnected.");
                            return;
                        }
                    }
                }

                match websocket.read() {
                    Ok(Message::Text(text)) => {
                        // Handle client commands
                        if let Ok(val) = serde_json::from_str::<serde_json::Value>(&text) {
                            if let Some(cmd) = val.get("cmd").and_then(|c| c.as_str()) {
                                match cmd {
                                    "reset_max" => {
                                        let mut cfg = dsp_config_client.write().unwrap();
                                        cfg.reset_max = true;
                                    }
                                    "set_avg" => {
                                        if let Some(alpha) = val.get("alpha").and_then(|a| a.as_f64()) {
                                            let mut cfg = dsp_config_client.write().unwrap();
                                            cfg.avg_alpha = (alpha as f32).clamp(0.001, 1.0);
                                        }
                                    }
                                    _ => {}
                                }
                            }
                        }
                    }
                    Ok(Message::Ping(data)) => {
                        let _ = websocket.send(Message::Pong(data));
                    }
                    Ok(Message::Close(_)) => {
                        println!("[WS] Client closed connection.");
                        return;
                    }
                    Err(tungstenite::Error::Io(ref io_err)) if io_err.kind() == std::io::ErrorKind::WouldBlock => {
                        thread::sleep(Duration::from_millis(5));
                    }
                    Err(e) => {
                        println!("[WS] Read error / disconnected: {:?}", e);
                        return;
                    }
                    _ => {}
                }
            }
        });
    }

    let _ = ftdi_thread.join();
    let _ = broadcast_thread.join();
    println!("Shutdown complete.");
}
