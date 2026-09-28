#!/usr/bin/env python3
"""
Graphical Real-Time FFT Spectrum Visualizer (FT232H + Matplotlib)
-----------------------------------------------------------------
Visualizes the 1024-point FFT spectrum streamed from the FPGA.
Displays:
  - 0 to 25 MHz (Nyquist frequency)
  - Real-time spectral peaks (e.g. 2.0 MHz sine peak)
"""

import sys
import time
import ctypes
import numpy as np

try:
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
except ImportError:
    sys.exit("Matplotlib is required for the GUI. Run: pip install matplotlib")

# Load libftdi1
try:
    lib = ctypes.CDLL("libftdi1.so.2")
except OSError:
    lib = ctypes.CDLL("libftdi1.so")

BITMODE_RESET  = 0x00
BITMODE_SYNCFF = 0x40
CHUNK_SIZE     = 65536
FS_HZ          = 50_000_000.0
FFT_SIZE       = 1024
NUM_BINS       = 512  # Display 0 to 25 MHz (half spectrum)
BIN_WIDTH_HZ   = FS_HZ / FFT_SIZE

lib.ftdi_new.restype = ctypes.c_void_p
lib.ftdi_free.argtypes = [ctypes.c_void_p]
lib.ftdi_set_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]
lib.ftdi_usb_open.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
lib.ftdi_usb_open.restype = ctypes.c_int
lib.ftdi_usb_close.argtypes = [ctypes.c_void_p]
lib.ftdi_set_bitmode.argtypes = [ctypes.c_void_p, ctypes.c_ubyte, ctypes.c_ubyte]
lib.ftdi_set_latency_timer.argtypes = [ctypes.c_void_p, ctypes.c_ubyte]
lib.ftdi_read_data_set_chunksize.argtypes = [ctypes.c_void_p, ctypes.c_uint]
lib.ftdi_read_data.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
lib.ftdi_read_data.restype = ctypes.c_int
lib.ftdi_get_error_string.argtypes = [ctypes.c_void_p]
lib.ftdi_get_error_string.restype = ctypes.c_char_p


def main():
    ctx = lib.ftdi_new()
    lib.ftdi_set_interface(ctx, 0)
    ret = lib.ftdi_usb_open(ctx, 0x0403, 0x6014)
    if ret < 0:
        err = lib.ftdi_get_error_string(ctx).decode('utf-8', errors='ignore')
        lib.ftdi_free(ctx)
        sys.exit(f"Failed to open FT232H: {err}")

    lib.ftdi_set_bitmode(ctx, 0x00, BITMODE_RESET)
    time.sleep(0.01)
    lib.ftdi_set_bitmode(ctx, 0xFF, BITMODE_SYNCFF)
    lib.ftdi_set_latency_timer(ctx, 2)
    CHUNK_SIZE = 256 * 1024
    lib.ftdi_read_data_set_chunksize(ctx, CHUNK_SIZE)

    raw_buf = (ctypes.c_ubyte * CHUNK_SIZE)()
    stream_buf = bytearray()
    current_frame = []
    in_frame = False
    latest_spectrum = np.zeros(NUM_BINS, dtype=np.float32)

    # Setup Matplotlib Figure
    freq_axis_mhz = np.linspace(0, (FS_HZ / 2) / 1e6, NUM_BINS)
    fig, ax = plt.subplots(figsize=(10, 5))
    line, = ax.plot(freq_axis_mhz, latest_spectrum, color='#00d2ff', lw=1.5)
    ax.set_xlim(0, 25.0)
    ax.set_ylim(0, 4200)
    ax.set_title("Cyclone IV Real-Time 1024-Point FFT Spectrum (FT232H Sync FIFO)", fontsize=12)
    ax.set_xlabel("Frequency (MHz)", fontsize=10)
    ax.set_ylabel("Power Amplitude (12-bit)", fontsize=10)
    ax.grid(True, linestyle="--", alpha=0.5)

    def update(frame):
        nonlocal in_frame, current_frame, latest_spectrum, stream_buf
        # Drain available USB packets
        for _ in range(3):
            n = lib.ftdi_read_data(ctx, raw_buf, CHUNK_SIZE)
            if n <= 0:
                break
            stream_buf.extend(raw_buf[:n])

            i = 0
            buf_len = len(stream_buf)
            while i < buf_len - 1:
                b0 = stream_buf[i]
                b1 = stream_buf[i + 1]
                tag = b1 & 0xF0

                if tag == 0xB0:
                    if in_frame and len(current_frame) >= 1000:
                        latest_spectrum = np.array(current_frame[:NUM_BINS], dtype=np.float32)
                    current_frame = [((b1 & 0x0F) << 8) | b0]
                    in_frame = True
                    i += 2
                elif tag == 0xA0 and in_frame:
                    current_frame.append(((b1 & 0x0F) << 8) | b0)
                    if len(current_frame) > 1024:
                        in_frame = False
                        current_frame = []
                    i += 2
                else:
                    i += 1

            del stream_buf[:i]

        line.set_ydata(latest_spectrum)
        peak_idx = np.argmax(latest_spectrum[1:]) + 1
        peak_mhz = freq_axis_mhz[peak_idx]
        ax.set_title(f"Live Spectrum: Peak @ {peak_mhz:.3f} MHz (Power: {latest_spectrum[peak_idx]:.0f})")
        return line,

    ani = FuncAnimation(fig, update, interval=30, blit=False)
    try:
        plt.show()
    finally:
        lib.ftdi_usb_close(ctx)
        lib.ftdi_free(ctx)


if __name__ == "__main__":
    main()
