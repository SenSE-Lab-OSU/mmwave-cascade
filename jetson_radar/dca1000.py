"""
dca1000.py - DCA1000EVM control over its UDP config port (replaces
DCA1000EVM_CLI_Control.exe, which is x86-only and cannot run on the Jetson).

Protocol (DCA1000EVM user guide / TI CLI):
  request : 0xA55A | cmd u16 | len u16 | data[len] | 0xEEAA   (all little-endian)
  response: 0xA55A | cmd u16 | status u16              | 0xEEAA
  status 0 = success (READ_FPGA_VERSION returns the version in the status word).

The FPGA keeps its configuration until the DCA1000 loses power, and re-sending
the same configuration is harmless, so configure() is safe to call every run.

Defaults below reproduce AM273X_Capture.json:
  raw logging, 4-lane LVDS, LVDS capture, Ethernet stream, 16-bit,
  packet delay 5 us, packet size 1470, FPGA 192.168.33.180, host 192.168.33.30.
Byte layouts verified against TI's rf_api.cpp / rf_api.h (mmWave Studio 3.0).
"""
import socket
import struct

# ---- command codes ----
CMD_RESET_FPGA          = 0x01
CMD_RESET_AR_DEV        = 0x02
CMD_CONFIG_FPGA_GEN     = 0x03
CMD_CONFIG_EEPROM       = 0x04
CMD_RECORD_START        = 0x05
CMD_RECORD_STOP         = 0x06
CMD_PLAYBACK_START      = 0x07
CMD_PLAYBACK_STOP       = 0x08
CMD_SYSTEM_CONNECT      = 0x09
CMD_SYSTEM_ERROR        = 0x0A
CMD_CONFIG_PACKET_DATA  = 0x0B
CMD_CONFIG_DATA_MODE    = 0x0C
CMD_INIT_FPGA_PLAYBACK  = 0x0D
CMD_READ_FPGA_VERSION   = 0x0E

# ---- CONFIG_FPGA_GEN field values ----
LOG_MODE_RAW, LOG_MODE_MULTI           = 1, 2
LVDS_4_LANE, LVDS_2_LANE               = 1, 2     # json lvdsMode 1 = 4 lanes
TRANSFER_LVDS_CAPTURE, TRANSFER_PLAYBACK = 1, 2
CAPTURE_SD_CARD, CAPTURE_ETHERNET      = 1, 2
FORMAT_12BIT, FORMAT_14BIT, FORMAT_16BIT = 1, 2, 3

HEADER = 0xA55A
FOOTER = 0xEEAA

# TI CLI values: MAX_BYTES_PER_PACKET = 1470 (defines.h),
# FPGA_CONFIG_DEFAULT_TIMER = 30 (globals.h).
PACKET_SIZE   = 1470
DEFAULT_TIMER = 30


class DCA1000Error(RuntimeError):
    pass


class DCA1000:
    def __init__(self, fpga_ip="192.168.33.180", host_ip="192.168.33.30",
                 config_port=4096, timeout=2.0, verbose=True):
        self.fpga = (fpga_ip, config_port)
        self.verbose = verbose
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host_ip, config_port))
        self.timeout = timeout
        self.sock.settimeout(timeout)

    def close(self):
        self.sock.close()

    # ---- low level ----
    @staticmethod
    def _packet(cmd, data=b""):
        return (struct.pack("<HHH", HEADER, cmd, len(data)) + data
                + struct.pack("<H", FOOTER))

    def drain_async(self):
        """Read and report any unsolicited SYSTEM_ERROR (0x0A) messages waiting
        on the config port. Called before each command and between responses."""
        self.sock.settimeout(0.0)
        try:
            while True:
                try:
                    resp, _ = self.sock.recvfrom(64)
                except (BlockingIOError, socket.timeout, OSError):
                    break
                self._report_if_async(resp)
        finally:
            self.sock.settimeout(self.timeout)

    def _report_if_async(self, resp):
        if len(resp) >= 8:
            hdr, rcmd, status, ftr = struct.unpack("<HHHH", resp[:8])
            if hdr == HEADER and ftr == FOOTER and rcmd == CMD_SYSTEM_ERROR:
                print(f"DCA1000: system error event 0x{status:04x}")
                return True
        return False

    def _command(self, cmd, data=b"", name=""):
        self.drain_async()
        self.sock.sendto(self._packet(cmd, data), self.fpga)
        for _ in range(8):   # skip async error events until our reply arrives
            try:
                resp, _ = self.sock.recvfrom(64)
            except socket.timeout:
                raise DCA1000Error(f"DCA1000: no response to {name or hex(cmd)} "
                                   f"(is it powered and on 192.168.33.180?)")
            if self._report_if_async(resp):
                continue
            if len(resp) < 8:
                raise DCA1000Error(f"DCA1000: short response to {name}: {resp.hex()}")
            hdr, rcmd, status, ftr = struct.unpack("<HHHH", resp[:8])
            if hdr != HEADER or ftr != FOOTER or rcmd != cmd:
                raise DCA1000Error(f"DCA1000: unexpected response to {name}: {resp.hex()}")
            return status
        raise DCA1000Error(f"DCA1000: too many async messages while waiting for {name}")

    def _check(self, cmd, data, name):
        status = self._command(cmd, data, name)
        if status != 0:
            raise DCA1000Error(f"DCA1000: {name} failed, status {status}")
        if self.verbose:
            print(f"DCA1000: {name} ok")

    # ---- commands ----
    def connect(self):
        self._check(CMD_SYSTEM_CONNECT, b"", "connect")

    def read_fpga_version(self):
        v = self._command(CMD_READ_FPGA_VERSION, b"", "read_fpga_version")
        major, minor, playback = v & 0x7F, (v >> 7) & 0x7F, (v >> 14) & 1
        if self.verbose:
            print(f"DCA1000: FPGA version {major}.{minor}"
                  f"{' (playback build)' if playback else ''}")
        return major, minor

    def reset_fpga(self):
        self._check(CMD_RESET_FPGA, b"", "reset_fpga")

    def config_fpga(self, log_mode=LOG_MODE_RAW, lvds_mode=LVDS_4_LANE,
                    transfer_mode=TRANSFER_LVDS_CAPTURE,
                    capture_mode=CAPTURE_ETHERNET, data_format=FORMAT_16BIT,
                    timer_s=DEFAULT_TIMER):
        data = bytes([log_mode, lvds_mode, transfer_mode, capture_mode,
                      data_format, timer_s])
        self._check(CMD_CONFIG_FPGA_GEN, data, "config_fpga")

    def config_packet(self, packet_size=PACKET_SIZE, delay_us=5):
        # FPGA delay units are 8 ns clocks: delay_us * 1000 / 8  (rf_api.cpp)
        delay = int(round(delay_us * 1000 / 8))
        data = struct.pack("<HHH", packet_size, delay, 0)
        self._check(CMD_CONFIG_PACKET_DATA, data, f"config_packet ({delay_us} us)")

    def start_record(self):
        self._check(CMD_RECORD_START, b"", "start_record")

    def stop_record(self):
        self._check(CMD_RECORD_STOP, b"", "stop_record")

    # ---- convenience: what start_capture.bat used to do ----
    def configure(self, packet_delay_us=5):
        self.connect()
        self.read_fpga_version()
        self.config_fpga()
        self.config_packet(delay_us=packet_delay_us)


if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser(description="DCA1000 control (Python)")
    ap.add_argument("cmd", choices=["configure", "start", "stop", "version", "reset"])
    a = ap.parse_args()
    d = DCA1000()
    try:
        {"configure": d.configure, "start": d.start_record, "stop": d.stop_record,
         "version": d.read_fpga_version, "reset": d.reset_fpga}[a.cmd]()
    finally:
        d.close()
