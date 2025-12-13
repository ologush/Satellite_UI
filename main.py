import tkinter as tk
from tkinter import ttk, messagebox

import serial
from serial.tools import list_ports


def get_serial_ports():
    # On macOS you’ll typically want /dev/cu.* (call-up devices) rather than /dev/tty.*
    ports = [p.device for p in list_ports.comports()]
    ports.sort()
    return ports


class YawControlApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Yaw Controller")
        self.geometry("600x220")
        self.resizable(False, False)

        self.ser = None

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="115200")
        self.status_var = tk.StringVar(value="Disconnected")

        self._build_ui()
        self.refresh_ports()

        self.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        pad = {"padx": 10, "pady": 8}

        # Row 0: Port selection + refresh
        ttk.Label(self, text="Serial Port:").grid(row=0, column=0, sticky="w", **pad)

        self.port_combo = ttk.Combobox(self, textvariable=self.port_var, state="readonly", width=28)
        self.port_combo.grid(row=0, column=1, sticky="w", **pad)

        ttk.Button(self, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, sticky="w", **pad)

        # Row 1: Baud rate + connect/disconnect
        ttk.Label(self, text="Baud:").grid(row=1, column=0, sticky="w", **pad)
        self.baud_entry = ttk.Entry(self, textvariable=self.baud_var, width=10)
        self.baud_entry.grid(row=1, column=1, sticky="w", **pad)

        self.connect_btn = ttk.Button(self, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=1, column=2, sticky="w", **pad)

        # Row 2: Command buttons
        self.yaw_plus_btn = ttk.Button(self, text="Yaw +10°", command=lambda: self.send_yaw(+10))
        self.yaw_minus_btn = ttk.Button(self, text="Yaw -10°", command=lambda: self.send_yaw(-10))
        self.yaw_minus_btn.grid(row=2, column=1, sticky="w", **pad)
        self.yaw_plus_btn.grid(row=2, column=2, sticky="w", **pad)

        # Row 3: Status
        ttk.Label(self, text="Status:").grid(row=3, column=0, sticky="w", **pad)
        ttk.Label(self, textvariable=self.status_var).grid(row=3, column=1, columnspan=2, sticky="w", **pad)

        self._set_connected_ui(False)

    def refresh_ports(self):
        ports = get_serial_ports()
        self.port_combo["values"] = ports

        # Keep current selection if still present; otherwise pick first
        current = self.port_var.get()
        if current in ports:
            self.port_var.set(current)
        elif ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("No port selected", "Select a serial port from the dropdown.")
            return

        try:
            baud = int(self.baud_var.get().strip())
        except ValueError:
            messagebox.showerror("Invalid baud", "Baud rate must be a number (e.g., 115200).")
            return

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1,         # read timeout (seconds)
                write_timeout=1     # write timeout (seconds)
            )
        except Exception as e:
            messagebox.showerror("Connection failed", f"Could not open {port}\n\n{e}")
            self.ser = None
            return

        self.status_var.set(f"Connected to {port} @ {baud}")
        self.connect_btn.config(text="Disconnect")
        self._set_connected_ui(True)

    def disconnect(self):
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        finally:
            self.ser = None

        self.status_var.set("Disconnected")
        self.connect_btn.config(text="Connect")
        self._set_connected_ui(False)

    def _set_connected_ui(self, connected: bool):
        state = "normal" if connected else "disabled"
        self.yaw_plus_btn.config(state=state)
        self.yaw_minus_btn.config(state=state)

    def send_yaw(self, delta_deg: int):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("Not connected", "Connect to a serial port first.")
            return

        # ✅ Define your device’s command format here.
        # Common simple format:
        #   "YAW:+10\n" or "YAW:-10\n"
        cmd = f"YAW:{delta_deg:+d}\n".encode("ascii")

        try:
            self.ser.write(cmd)
            self.ser.flush()
            self.status_var.set(f"Sent: {cmd.decode('ascii').strip()}")
        except Exception as e:
            messagebox.showerror("Send failed", str(e))
            self.disconnect()

    def on_close(self):
        self.disconnect()
        self.destroy()


if __name__ == "__main__":
    app = YawControlApp()
    app.mainloop()
