#!/usr/bin/env python3
"""
FT232H link integrity check. FPGA must send the 16-bit counter 0,1,2,... via
ft_pattern_src.v (little endian: low byte first). Any lost / duplicated / shifted
byte shows up as a break in the +1 sequence.

    python3 ft_check.py [seconds]      (default 30)
    python3 ft_check.py --selftest     (checker logic only, no hardware)
"""
import sys, time, ctypes
import numpy as np


class Checker:
    def __init__(self):
        self.tail = b""          # leftover bytes < 1 word
        self.last = None         # last word of previous chunk
        self.offset = None       # byte alignment (0/1), locked on first data
        self.pend = b""
        self.words = 0
        self.errors = 0
        self.first_err = []

    @staticmethod
    def _score(b, off):
        n = (len(b) - off) // 2
        if n < 8:
            return -1
        w = np.frombuffer(b[off:off + 2 * n], dtype="<u2")
        return int(np.count_nonzero(((w[1:] - w[:-1]) & 0xFFFF) == 1))

    def feed(self, data):
        if self.offset is None:                    # pick alignment from first 8 KB
            self.pend += data
            if len(self.pend) < 8192:
                return
            self.offset = 0 if self._score(self.pend, 0) >= self._score(self.pend, 1) else 1
            data, self.pend = self.pend[self.offset:], b""
        buf = self.tail + data
        n = len(buf) // 2
        self.tail = buf[2 * n:]
        if n == 0:
            return
        w = np.frombuffer(buf[:2 * n], dtype="<u2").astype(np.int32)
        if self.last is not None:
            w = np.concatenate(([self.last], w))
            base = self.words - 1
        else:
            base = self.words
        bad = np.nonzero(((w[1:] - w[:-1]) & 0xFFFF) != 1)[0]
        self.errors += len(bad)
        for i in bad[: max(0, 10 - len(self.first_err))]:
            self.first_err.append((base + int(i) + 1, int(w[i]), int(w[i + 1])))
        self.last = int(w[-1])
        self.words += n


def selftest():
    rng = np.random.default_rng(1)
    seq = (np.arange(200000) & 0xFFFF).astype("<u2").tobytes()
    c = Checker(); p = 0
    while p < len(seq):                            # clean stream, odd chunk sizes, odd start
        k = int(rng.integers(1000, 70000)); c.feed(seq[1 + p:1 + p + k]); p += k
    assert c.errors == 0, c.errors
    bad = bytearray(seq); del bad[100000]          # one lost byte
    c = Checker(); p = 0
    while p < len(bad):
        k = int(rng.integers(1000, 70000)); c.feed(bytes(bad[p:p + k])); p += k
    assert c.errors >= 1, "slip not detected"
    print("selftest OK (clean: 0 errors, 1 lost byte: detected)")


def main():
    if "--selftest" in sys.argv:
        return selftest()
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
    try:
        lib = ctypes.CDLL("libftdi1.so.2")
    except OSError:
        lib = ctypes.CDLL("libftdi1.so")
    lib.ftdi_new.restype = ctypes.c_void_p
    for f in ("ftdi_usb_open", "ftdi_set_bitmode", "ftdi_set_latency_timer",
              "ftdi_read_data_set_chunksize", "ftdi_read_data", "ftdi_set_interface"):
        getattr(lib, f).restype = ctypes.c_int
    lib.ftdi_usb_open.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
    lib.ftdi_set_bitmode.argtypes = [ctypes.c_void_p, ctypes.c_ubyte, ctypes.c_ubyte]
    lib.ftdi_set_latency_timer.argtypes = [ctypes.c_void_p, ctypes.c_ubyte]
    lib.ftdi_read_data_set_chunksize.argtypes = [ctypes.c_void_p, ctypes.c_uint]
    lib.ftdi_read_data.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_ubyte), ctypes.c_int]
    lib.ftdi_set_interface.argtypes = [ctypes.c_void_p, ctypes.c_int]

    ctx = lib.ftdi_new()
    lib.ftdi_set_interface(ctx, 0)
    if lib.ftdi_usb_open(ctx, 0x0403, 0x6014) < 0:
        sys.exit("cannot open FT232H (close spectrum-rx first / check udev)")
    lib.ftdi_set_bitmode(ctx, 0x00, 0x00); time.sleep(0.01)
    lib.ftdi_set_bitmode(ctx, 0xFF, 0x40)
    lib.ftdi_set_latency_timer(ctx, 2)
    lib.ftdi_read_data_set_chunksize(ctx, 256 * 1024)

    buf = (ctypes.c_ubyte * (256 * 1024))()
    chk, t0, tl, nbytes = Checker(), time.time(), time.time(), 0
    raw_head = b""
    try:
      while time.time() - t0 < secs:
        n = lib.ftdi_read_data(ctx, buf, len(buf))
        if n <= 0:
            time.sleep(0.0005); continue
        nbytes += n
        d = bytes(buf[:n])
        if len(raw_head) < 64:
            raw_head += d[:64 - len(raw_head)]
        chk.feed(d)
        if time.time() - tl > 1.0:
            tl = time.time()
            print(f"\r{nbytes/1e6:9.1f} MB  words {chk.words:>12}  ERRORS {chk.errors}", end="", flush=True)
    except KeyboardInterrupt:
        pass
    dt = time.time() - t0
    print(f"\n{nbytes/dt/1e6:.1f} MB/s, {chk.words} words, {chk.errors} sequence errors")
    for e in chk.first_err:
        print(f"  word #{e[0]}: {e[1]:#06x} -> {e[2]:#06x}")
    print("first 64 raw bytes:", raw_head.hex(" "))
    print("PASS" if chk.errors == 0 and chk.words > 1_000_000 else "FAIL / too little data")
    lib.ftdi_set_bitmode(ctx, 0xFF, 0x00)


if __name__ == "__main__":
    main()
