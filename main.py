import tkinter as tk
from tkinter import ttk, scrolledtext
import threading
import queue
import struct
import serial
from serial.tools import list_ports

# ── Command IDs ───────────────────────────────────────────────────────────────
CMD_SET_ADCS_MODE         = 0x01
CMD_SET_ADCS_TARGET       = 0x02
CMD_GET_BASE_SENSOR_DATA  = 0x03
CMD_GET_SAT_SENSOR_DATA   = 0x04
CMD_RESP_SAT_SENSOR_DATA  = 0x10
CMD_RESP_BASE_SENSOR_DATA = 0x11
CMD_RESP_ACK              = 0x20
CMD_RESP_NACK             = 0x21

# Total byte length of each response packet (including the leading command byte)
RESPONSE_LENGTHS = {
    CMD_RESP_BASE_SENSOR_DATA: 9,   # 0x11 + temp(4) + pot(4)
    CMD_RESP_SAT_SENSOR_DATA:  21,  # 0x10 + sat_temp(4) + pot(4) + yaw(4) + yaw_rate(4) + imu_temp(4)
    CMD_RESP_ACK:              2,   # 0x20 + original_cmd
    CMD_RESP_NACK:             2,   # 0x21 + original_cmd
}

ADCS_MODES = {
    0: "Attitude Target",
    1: "Spin Rate Target",
    2: "Attitude Control Off",
}

POLL_INTERVAL_MS  = 2000   # poll each data source every 2 s
SAT_POLL_OFFSET_MS = 1000  # stagger satellite poll 1 s after base poll


def get_serial_ports():
    ports = [p.device for p in list_ports.comports()]
    # On macOS, prefer /dev/cu.* (callout) over /dev/tty.* (tty line discipline
    # can silently drop binary data). Filter out tty.* if cu.* equivalents exist.
    cu_ports  = [p for p in ports if p.startswith("/dev/cu.")]
    tty_ports = [p for p in ports if p.startswith("/dev/tty.")]
    other     = [p for p in ports if not p.startswith("/dev/cu.") and not p.startswith("/dev/tty.")]
    # Drop /dev/tty.* that have a /dev/cu.* counterpart
    tty_keep  = [p for p in tty_ports if p.replace("/dev/tty.", "/dev/cu.") not in cu_ports]
    result = sorted(cu_ports + tty_keep + other)
    return result


def unpack_float_le(data: bytes, offset: int) -> float:
    return struct.unpack_from("<f", data, offset)[0]


class SatelliteGroundUI(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Satellite Ground Control")
        self.resizable(True, True)

        self.ser = None
        self.rx_queue: queue.Queue = queue.Queue()
        self.tx_queue: queue.Queue = queue.Queue()
        self.stop_io = threading.Event()
        self.reader_thread = None
        self.writer_thread = None

        # Connection vars
        self.port_var   = tk.StringVar()
        self.baud_var   = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="Disconnected")

        # ADCS target entry
        self.target_var = tk.StringVar(value="0")

        # Base station sensor display vars
        self.base_temp_var = tk.StringVar(value="—")
        self.base_pot_var  = tk.StringVar(value="—")

        # Satellite sensor display vars
        self.sat_temp_var      = tk.StringVar(value="—")
        self.sat_pot_var       = tk.StringVar(value="—")
        self.sat_yaw_var       = tk.StringVar(value="—")
        self.sat_yaw_rate_var  = tk.StringVar(value="—")
        self.sat_imu_temp_var  = tk.StringVar(value="—")

        self._build_ui()
        self.refresh_ports()
        self.protocol("WM_DELETE_WINDOW", self.on_close)

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        self.columnconfigure(0, weight=1)
        self.rowconfigure(2, weight=1)

        pad = {"padx": 8, "pady": 6}

        # ── Connection bar ────────────────────────────────────────────────────
        conn = ttk.LabelFrame(self, text="Connection")
        conn.grid(row=0, column=0, sticky="ew", **pad)
        conn.columnconfigure(6, weight=1)

        ttk.Label(conn, text="Port:").grid(row=0, column=0, sticky="w", padx=6, pady=4)
        self.port_combo = ttk.Combobox(conn, textvariable=self.port_var,
                                       state="readonly", width=26)
        self.port_combo.grid(row=0, column=1, sticky="w", padx=6, pady=4)
        ttk.Button(conn, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=4, pady=4)

        ttk.Label(conn, text="Baud:").grid(row=0, column=3, sticky="e", padx=(16, 4), pady=4)
        ttk.Entry(conn, textvariable=self.baud_var, width=9).grid(
            row=0, column=4, sticky="w", padx=4, pady=4)

        self.connect_btn = ttk.Button(conn, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=8, pady=4)

        ttk.Label(conn, textvariable=self.status_var, foreground="gray").grid(
            row=0, column=6, sticky="w", padx=12, pady=4)

        # ── Sensor panels ─────────────────────────────────────────────────────
        sensors = ttk.Frame(self)
        sensors.grid(row=1, column=0, sticky="ew", **pad)
        sensors.columnconfigure(0, weight=1)
        sensors.columnconfigure(1, weight=1)

        VALUE_FONT = ("Courier", 12, "bold")

        # Base station
        base = ttk.LabelFrame(sensors, text="Base Station Sensors")
        base.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        base.columnconfigure(1, weight=1)

        ttk.Label(base, text="Temperature:").grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(base, textvariable=self.base_temp_var, font=VALUE_FONT).grid(
            row=0, column=1, sticky="w", **pad)

        ttk.Label(base, text="Potentiometer:").grid(row=1, column=0, sticky="w", **pad)
        ttk.Label(base, textvariable=self.base_pot_var, font=VALUE_FONT).grid(
            row=1, column=1, sticky="w", **pad)

        # Satellite
        sat = ttk.LabelFrame(sensors, text="Satellite Sensors")
        sat.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        sat.columnconfigure(1, weight=1)

        ttk.Label(sat, text="Sat Temp:").grid(row=0, column=0, sticky="w", **pad)
        ttk.Label(sat, textvariable=self.sat_temp_var, font=VALUE_FONT).grid(
            row=0, column=1, sticky="w", **pad)

        ttk.Label(sat, text="Potentiometer:").grid(row=1, column=0, sticky="w", **pad)
        ttk.Label(sat, textvariable=self.sat_pot_var, font=VALUE_FONT).grid(
            row=1, column=1, sticky="w", **pad)

        ttk.Label(sat, text="Yaw:").grid(row=2, column=0, sticky="w", **pad)
        ttk.Label(sat, textvariable=self.sat_yaw_var, font=VALUE_FONT).grid(
            row=2, column=1, sticky="w", **pad)

        ttk.Label(sat, text="Yaw Rate:").grid(row=3, column=0, sticky="w", **pad)
        ttk.Label(sat, textvariable=self.sat_yaw_rate_var, font=VALUE_FONT).grid(
            row=3, column=1, sticky="w", **pad)

        ttk.Label(sat, text="IMU Temp:").grid(row=4, column=0, sticky="w", **pad)
        ttk.Label(sat, textvariable=self.sat_imu_temp_var, font=VALUE_FONT).grid(
            row=4, column=1, sticky="w", **pad)

        # ── ADCS controls ─────────────────────────────────────────────────────
        adcs = ttk.LabelFrame(self, text="ADCS Control")
        adcs.grid(row=2, column=0, sticky="ew", **pad)

        ttk.Label(adcs, text="Mode:").grid(row=0, column=0, sticky="w", padx=8, pady=6)
        for col, (mode_id, mode_name) in enumerate(ADCS_MODES.items(), start=1):
            ttk.Button(adcs, text=mode_name,
                       command=lambda m=mode_id: self.send_adcs_mode(m)).grid(
                row=0, column=col, padx=4, pady=6)

        ttk.Label(adcs, text="Target:").grid(row=1, column=0, sticky="w", padx=8, pady=6)
        ttk.Entry(adcs, textvariable=self.target_var, width=6).grid(
            row=1, column=1, sticky="w", padx=4, pady=6)
        ttk.Button(adcs, text="Set Target", command=self.send_adcs_target).grid(
            row=1, column=2, padx=4, pady=6)

        # ── Log ───────────────────────────────────────────────────────────────
        log_frame = ttk.LabelFrame(self, text="Log")
        log_frame.grid(row=3, column=0, sticky="nsew", **pad)
        self.rowconfigure(3, weight=1)

        self.log_box = scrolledtext.ScrolledText(
            log_frame, height=8, state="disabled",
            font=("Courier", 10), wrap=tk.WORD)
        self.log_box.pack(fill="both", expand=True, padx=4, pady=4)

    # ── Port helpers ──────────────────────────────────────────────────────────

    def refresh_ports(self):
        ports = get_serial_ports()
        self.port_combo["values"] = ports
        current = self.port_var.get()
        if current in ports:
            self.port_var.set(current)
        elif ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    # ── Connection ────────────────────────────────────────────────────────────

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            self.log("No port selected.")
            return
        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            self.log("Invalid baud rate.")
            return
        try:
            self.ser = serial.Serial(port=port, baudrate=baud,
                                     timeout=1, write_timeout=1)
        except Exception as e:
            self.log(f"Connection failed: {e}")
            self.ser = None
            return

        self.stop_io.clear()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self.reader_thread.start()
        self.writer_thread.start()
        self.after(50, self._process_rx_queue)

        self.status_var.set(f"Connected  {port} @ {baud}")
        self.connect_btn.config(text="Disconnect")
        self.log(f"Connected to {port} @ {baud}")
        self._start_polls()

    def disconnect(self):
        self.stop_io.set()
        self.tx_queue.put(None)  # unblock writer thread
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        finally:
            self.ser = None
        self.status_var.set("Disconnected")
        self.connect_btn.config(text="Connect")
        self.log("Disconnected.")

    # ── Polling ───────────────────────────────────────────────────────────────

    def _start_polls(self):
        self._poll_base()
        self.after(SAT_POLL_OFFSET_MS, self._poll_sat)

    def _poll_base(self):
        if not (self.ser and self.ser.is_open):
            return
        self._send_bytes(bytes([CMD_GET_BASE_SENSOR_DATA]))
        self.after(POLL_INTERVAL_MS, self._poll_base)

    def _poll_sat(self):
        if not (self.ser and self.ser.is_open):
            return
        self._send_bytes(bytes([CMD_GET_SAT_SENSOR_DATA]))
        self.after(POLL_INTERVAL_MS, self._poll_sat)

    # ── ADCS commands ─────────────────────────────────────────────────────────

    def send_adcs_mode(self, mode: int):
        if not (self.ser and self.ser.is_open):
            self.log("Not connected.")
            return
        self._send_bytes(bytes([CMD_SET_ADCS_MODE, mode]))
        self.log(f"TX  SET_ADCS_MODE  mode={mode} ({ADCS_MODES.get(mode, '?')})")

    def send_adcs_target(self):
        if not (self.ser and self.ser.is_open):
            self.log("Not connected.")
            return
        try:
            target = int(self.target_var.get().strip())
            if not (0 <= target <= 255):
                raise ValueError
        except ValueError:
            self.log("Target must be an integer 0–255.")
            return
        self._send_bytes(bytes([CMD_SET_ADCS_TARGET, target]))
        self.log(f"TX  SET_ADCS_TARGET  target={target}")

    # ── Raw send (non-blocking — enqueues for writer thread) ─────────────────

    def _send_bytes(self, data: bytes):
        self.tx_queue.put(data)

    # ── Writer thread (background) ────────────────────────────────────────────

    def _writer_loop(self):
        while not self.stop_io.is_set():
            try:
                data = self.tx_queue.get(timeout=1)
                if data is None:
                    break
                if self.ser and self.ser.is_open:
                    self.ser.write(data)
                    self.ser.flush()
                    self.rx_queue.put(f"__RAW__:TX {data.hex(' ').upper()}")
            except queue.Empty:
                continue
            except Exception as e:
                self.rx_queue.put(f"__ERROR__:{e}")
                break

    # ── Reader thread (background) ────────────────────────────────────────────

    def _reader_loop(self):
        buf = bytearray()
        while not self.stop_io.is_set():
            try:
                if not (self.ser and self.ser.is_open):
                    break
                chunk = self.ser.read(64)
                if not chunk:
                    continue
                self.rx_queue.put(f"__RAW__:{chunk.hex(' ').upper()}")
                buf.extend(chunk)

                # Parse as many complete packets as possible
                while buf:
                    cmd_id = buf[0]
                    pkt_len = RESPONSE_LENGTHS.get(cmd_id)
                    if pkt_len is None:
                        # Unknown leading byte — discard and re-sync
                        self.rx_queue.put(f"__RAW__:  discard 0x{cmd_id:02X}")
                        buf = buf[1:]
                        continue
                    if len(buf) < pkt_len:
                        break  # wait for more data
                    packet = bytes(buf[:pkt_len])
                    buf = buf[pkt_len:]
                    self.rx_queue.put(packet)

            except Exception as e:
                self.rx_queue.put(f"__ERROR__:reader: {e}")
                break

    # ── Rx processing (main thread) ───────────────────────────────────────────

    def _process_rx_queue(self):
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, str) and item.startswith("__ERROR__:"):
                    self.log(f"Serial error: {item[len('__ERROR__:'):]}")
                    self.disconnect()
                    return
                if isinstance(item, str) and item.startswith("__RAW__:"):
                    self.log(f"RX raw: {item[8:]}")
                    continue
                self._handle_packet(item)
        except queue.Empty:
            pass
        if self.ser and self.ser.is_open:
            self.after(50, self._process_rx_queue)

    def _handle_packet(self, pkt: bytes):
        if not pkt:
            return
        cmd_id = pkt[0]

        if cmd_id == CMD_RESP_BASE_SENSOR_DATA and len(pkt) >= 9:
            temp = unpack_float_le(pkt, 1)
            pot  = unpack_float_le(pkt, 5)
            self.base_temp_var.set(f"{temp:.2f} °C")
            self.base_pot_var.set(f"{pot:.1f} %")
            return

        if cmd_id == CMD_RESP_SAT_SENSOR_DATA and len(pkt) >= 21:
            self.log(f"SAT PKT [{len(pkt)}B]: {pkt.hex(' ').upper()}")
            sat_temp  = unpack_float_le(pkt, 1)
            pot       = unpack_float_le(pkt, 5)
            yaw       = unpack_float_le(pkt, 9)
            yaw_rate  = unpack_float_le(pkt, 13)
            imu_temp  = unpack_float_le(pkt, 17)
            self.sat_temp_var.set(f"{sat_temp:.2f} °C")
            self.sat_pot_var.set(f"{pot:.1f} %")
            self.sat_yaw_var.set(f"{yaw:.4f} rad")
            self.sat_yaw_rate_var.set(f"{yaw_rate:.2f} °/s")
            self.sat_imu_temp_var.set(f"{imu_temp:.2f} °C")
            return

        if cmd_id == CMD_RESP_ACK and len(pkt) >= 2:
            self.log(f"ACK   cmd=0x{pkt[1]:02X}")
            return

        if cmd_id == CMD_RESP_NACK and len(pkt) >= 2:
            self.log(f"NACK  cmd=0x{pkt[1]:02X}")
            return

        self.log(f"Unknown packet: {pkt.hex(' ').upper()}")

    # ── Log ───────────────────────────────────────────────────────────────────

    def log(self, msg: str):
        self.log_box.config(state="normal")
        self.log_box.insert(tk.END, msg + "\n")
        self.log_box.see(tk.END)
        self.log_box.config(state="disabled")

    # ── Close ─────────────────────────────────────────────────────────────────

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = SatelliteGroundUI()
    app.mainloop()
