#!/usr/bin/env python3
"""
Real-Time FFT Spectrum Receiver for FT232H (Synchronous 245 FIFO)
------------------------------------------------------------------
Reads 1024-point FFT frames streamed from the Cyclone IV FPGA.
Data format per 16-bit word:
  - Byte 0: sample[7:0]  (low byte)
  - Byte 1: sample[15:8] (high byte: tag in top 4 bits, power in low 4 bits)
    * (Byte 1 & 0xF0) == 0xB0 -> Bin 0 (Start of Frame / SOP)
    * (Byte 1 & 0xF0) == 0xA0 -> Bins 1..1023
    * Power Value = ((Byte 1 & 0x0F) << 8) | Byte 0   (0 .. 4095)

Resolution:
  - Fs = 50.0 MHz
  - N = 1024 points
  - Bin resolution = 50 MHz / 1024 = 48.828 kHz per bin
"""

import sys
import time
import ctypes
import numpy as np

# Load system libftdi1 (standard on Fedora / Ubuntu)
try:
    lib = ctypes.CDLL("libftdi1.so.2")
except OSError:
    try:
        lib = ctypes.CDLL("libftdi1.so")
    except OSError:
        sys.exit("Error: Could not load libftdi1.so. Please install libftdi (e.g., sudo dnf install libftdi).")

# FTDI definitions
BITMODE_RESET  = 0x00
BITMODE_SYNCFF = 0x40  # 245 Synchronous FIFO mode

# Function signatures
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

CHUNK_SIZE = 65536
FS_HZ = 50_000_000.0
FFT_SIZE = 1024
BIN_WIDTH_HZ = FS_HZ / FFT_SIZE  # ~48828.125 Hz


def open_ft232h(vendor=0x0403, product=0x6014):
    ctx = lib.ftdi_new()
    if not ctx:
        raise RuntimeError("Failed to allocate FTDI context")

    lib.ftdi_set_interface(ctx, 0)
    ret = lib.ftdi_usb_open(ctx, vendor, product)
    if ret < 0:
        err = lib.ftdi_get_error_string(ctx).decode('utf-8', errors='ignore')
        lib.ftdi_free(ctx)
        raise RuntimeError(f"Failed to open FT232H (0x{vendor:04x}:0x{product:04x}): {err}\n"
                           f"Hint: Check USB permissions (udev rules) or run with sudo.")

    # Configure 245 Synchronous FIFO mode
    lib.ftdi_set_bitmode(ctx, 0x00, BITMODE_RESET)
    time.sleep(0.01)
    lib.ftdi_set_bitmode(ctx, 0xFF, BITMODE_SYNCFF)
    lib.ftdi_set_latency_timer(ctx, 2)
    lib.ftdi_read_data_set_chunksize(ctx, CHUNK_SIZE)
    return ctx


def main():
    print("=" * 65)
    print(" Cyclone IV Real-Time FFT Spectrum Receiver (FT232H)")
    print("=" * 65)
    print(f"Sampling Rate : {FS_HZ / 1e6:.1f} MHz")
    print(f"FFT Size      : {FFT_SIZE} points")
    print(f"Resolution    : {BIN_WIDTH_HZ / 1e3:.2f} kHz / bin")
    print("=" * 65)

    try:
        ctx = open_ft232h()
        print("Connected to FT232H in Synchronous FIFO mode.\n")
    except Exception as e:
        print(f"Error: {e}")
        return

    CHUNK_SIZE = 256 * 1024
    raw_buf = (ctypes.c_ubyte * CHUNK_SIZE)()
    stream_buf = bytearray()
    current_frame = []
    in_frame = False

    last_print = time.time()
    frames_received = 0
    total_bytes = 0

    try:
        while True:
            n = lib.ftdi_read_data(ctx, raw_buf, CHUNK_SIZE)
            if n <= 0:
                time.sleep(0.001)
                continue

            total_bytes += n
            stream_buf.extend(raw_buf[:n])

            i = 0
            buf_len = len(stream_buf)
            while i < buf_len - 1:
                b0 = stream_buf[i]
                b1 = stream_buf[i + 1]
                tag = b1 & 0xF0

                if tag == 0xB0:
                    # SOP (Bin 0: Start of new FFT frame)
                    if in_frame and len(current_frame) >= 1000:
                        frames_received += 1
                        spectrum = np.array(current_frame[:1024], dtype=np.uint16)

                        # Detect peak in positive spectrum (bins 1..511: 0 to 25 MHz)
                        half_spectrum = spectrum[1:512]
                        peak_bin = np.argmax(half_spectrum) + 1
                        peak_val = spectrum[peak_bin]
                        peak_freq_mhz = (peak_bin * BIN_WIDTH_HZ) / 1e6

                        # Print update every ~100ms
                        now = time.time()
                        if now - last_print >= 0.1:
                            dt = now - last_print
                            fps = frames_received / dt
                            mb_s = (total_bytes / (1024 * 1024)) / dt

                            bar_len = int((peak_val / 4095.0) * 30)
                            bar = "#" * bar_len + "-" * (30 - bar_len)

                            print(f"\r[Peak: {peak_freq_mhz:6.3f} MHz (Bin {peak_bin:3d}) | "
                                  f"Pwr: {peak_val:4d}/4095 |{bar}| "
                                  f"{fps:5.0f} fps | {mb_s:4.1f} MB/s]", end="", flush=True)

                            frames_received = 0
                            total_bytes = 0
                            last_print = now

                    # Start new frame
                    val = ((b1 & 0x0F) << 8) | b0
                    current_frame = [val]
                    in_frame = True
                    i += 2

                elif tag == 0xA0 and in_frame:
                    val = ((b1 & 0x0F) << 8) | b0
                    current_frame.append(val)
                    if len(current_frame) > 1024:
                        in_frame = False
                        current_frame = []
                    i += 2

                else:
                    i += 1

            del stream_buf[:i]

    except KeyboardInterrupt:
        print("\n\nExiting...")
    finally:
        lib.ftdi_usb_close(ctx)
        lib.ftdi_free(ctx)
        print("Device closed.")


if __name__ == "__main__":
    main()
