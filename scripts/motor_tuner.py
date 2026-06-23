#!/usr/bin/env python3
"""
motor_tuner.py -- Interactive terminal interface for AK40-10 motor control.

Connects to the STM32F446RE over USB Serial (USART2 via ST-Link virtual COM)
and lets you command individual motors and watch motor state feedback in real time.

Usage:
    python motor_tuner.py              # auto-detect STM32 COM port
    python motor_tuner.py COM3         # specify port explicitly

Requirements:
    pip install pyserial

Motor state is printed as it arrives from the STM32 in the background.
All position values are in radians. Velocity in rad/s. Torque in Nm.

Firmware command protocol (CubeIDE / HAL build):
    enable <id>        -- Enter MIT mode on motor
    disable <id>       -- Exit MIT mode on motor
    zero <id>          -- Set zero position (writes to motor flash)
    pos <id> <rad>     -- Move to position in radians
    stop               -- Disable all active motors
    status             -- Print firmware status

Note: kp and kd gains are set in firmware (ak40_driver.h constants).
      To change gains, edit AK40_KP_DEFAULT / AK40_KD_DEFAULT and reflash.
"""

import sys
import time
import threading
import serial
import serial.tools.list_ports

HELP_TEXT = """
Commands
--------
  motor <id>               Select active motor (1-12)
  enable                   Enter MIT mode on active motor
  disable                  Exit MIT mode on active motor
  zero                     Set zero position (writes to motor flash -- use carefully)
  goto <pos>               Move to position [rad]  (alias: pos)
  stop                     Disable all active motors
  status                   Request firmware status
  state                    Print last received motor state
  help                     Show this message
  quit                     Disable active motor and disconnect

Note: kp/kd gains are fixed in firmware. Edit AK40_KP_DEFAULT / AK40_KD_DEFAULT
      in ak40_driver.h and reflash to change them.
"""


class MotorTuner:
    """Manages the serial link to the STM32 and motor command state."""

    def __init__(self, port: str, baud: int = 115200):
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.motor_id: int = 1
        self._last_state: dict[int, tuple[float, float, float]] = {}
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _send(self, text: str) -> None:
        self.ser.write((text + "\n").encode())

    def _read_loop(self) -> None:
        """Background thread: reads lines from STM32 and stores state."""
        while not self._stop.is_set():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                if line.startswith("STATE "):
                    # STATE <id> <pos_rad> <vel_rad_s> <torque_Nm>
                    parts = line.split()
                    if len(parts) == 5:
                        mid = int(parts[1])
                        with self._lock:
                            self._last_state[mid] = (
                                float(parts[2]),
                                float(parts[3]),
                                float(parts[4]),
                            )
                else:
                    # Firmware log lines ([ok], [error], [boot]) go straight to console
                    print(f"\r  {line}")
                    print(f"\rmotor{self.motor_id}> ", end="", flush=True)
            except Exception:
                pass

    # ------------------------------------------------------------------
    # Motor commands -- match CubeIDE firmware protocol exactly
    # ------------------------------------------------------------------

    def enable(self, mid: int | None = None) -> None:
        self._send(f"enable {mid or self.motor_id}")

    def disable(self, mid: int | None = None) -> None:
        self._send(f"disable {mid or self.motor_id}")

    def set_zero(self, mid: int | None = None) -> None:
        self._send(f"zero {mid or self.motor_id}")

    def send_pos(self, pos: float, mid: int | None = None) -> None:
        m = mid or self.motor_id
        self._send(f"pos {m} {pos:.4f}")

    def stop_all(self) -> None:
        self._send("stop")

    def request_status(self) -> None:
        self._send("status")

    # ------------------------------------------------------------------
    # State display
    # ------------------------------------------------------------------

    def print_state(self) -> None:
        with self._lock:
            entry = self._last_state.get(self.motor_id)
        if entry:
            pos, vel, torque = entry
            print(
                f"  motor {self.motor_id}: "
                f"pos={pos:+.4f} rad  "
                f"vel={vel:+.4f} rad/s  "
                f"torque={torque:+.4f} Nm"
            )
        else:
            print(f"  motor {self.motor_id}: no state received yet")

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------

    def close(self) -> None:
        self._stop.set()
        try:
            self.ser.close()
        except Exception:
            pass


# -----------------------------------------------------------------------
# Port detection
# -----------------------------------------------------------------------

def find_stm32_port() -> str:
    """Return the first likely STM32 COM port, or ask the user to pick one."""
    ports = list(serial.tools.list_ports.comports())
    stm_keywords = ("STM", "ST-Link", "STMicro", "Virtual COM")
    stm_ports = [
        p for p in ports
        if any(kw.lower() in p.description.lower() for kw in stm_keywords)
    ]
    if stm_ports:
        print(f"Found STM32 on {stm_ports[0].device} ({stm_ports[0].description})")
        return stm_ports[0].device

    if not ports:
        raise RuntimeError(
            "No serial ports found. Is the STM32 plugged in via USB?"
        )

    print("No STM32 port auto-detected. Available ports:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  {p.description}")
    idx = int(input("Select port number: ").strip())
    return ports[idx].device


# -----------------------------------------------------------------------
# Main REPL
# -----------------------------------------------------------------------

def main() -> None:
    port = sys.argv[1] if len(sys.argv) > 1 else find_stm32_port()

    print(f"Connecting to {port} at 115200 baud...")
    tuner = MotorTuner(port)
    time.sleep(2.5)  # wait for STM32 boot messages

    print(f"Ready.  Active motor: {tuner.motor_id}")
    print("Type 'help' for a list of commands.\n")

    try:
        while True:
            try:
                raw = input(f"motor{tuner.motor_id}> ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            parts = raw.split()
            cmd = parts[0].lower()

            # --- quit ---
            if cmd == "quit":
                print(f"Disabling motor {tuner.motor_id}...")
                tuner.disable()
                time.sleep(0.2)
                break

            # --- help ---
            elif cmd == "help":
                print(HELP_TEXT)

            # --- motor <id> ---
            elif cmd == "motor":
                if len(parts) < 2:
                    print("  Usage: motor <id>")
                else:
                    mid = int(parts[1])
                    if 1 <= mid <= 12:
                        tuner.motor_id = mid
                        print(f"  Active motor: {mid}")
                    else:
                        print("  Motor ID must be 1-12")

            # --- enable ---
            elif cmd == "enable":
                tuner.enable()
                print(f"  enable sent to motor {tuner.motor_id}")

            # --- disable ---
            elif cmd == "disable":
                tuner.disable()
                print(f"  disable sent to motor {tuner.motor_id}")

            # --- zero ---
            elif cmd == "zero":
                confirm = input(
                    "  This writes to motor flash and shifts the zero reference.\n"
                    "  Type 'yes' to confirm: "
                ).strip()
                if confirm == "yes":
                    tuner.set_zero()
                    print("  Zero position set.")
                else:
                    print("  Cancelled.")

            # --- goto / pos <pos> ---
            elif cmd in ("goto", "pos"):
                if len(parts) < 2:
                    print("  Usage: goto <pos_rad>")
                else:
                    pos = float(parts[1])
                    tuner.send_pos(pos)
                    time.sleep(0.08)
                    tuner.print_state()

            # --- stop ---
            elif cmd == "stop":
                tuner.stop_all()
                print("  stop sent (all motors disabled)")

            # --- status ---
            elif cmd == "status":
                tuner.request_status()

            # --- state ---
            elif cmd == "state":
                tuner.print_state()

            else:
                print(f"  Unknown command '{cmd}'. Type 'help'.")

    finally:
        tuner.close()
        print("Disconnected.")


if __name__ == "__main__":
    main()
