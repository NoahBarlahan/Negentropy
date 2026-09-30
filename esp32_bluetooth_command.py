r"""Send one Bluetooth-serial command to ESP32ServoBluetooth.ino.

Examples (PowerShell):
  .\.venv\Scripts\python.exe .\esp32_bluetooth_command.py --list
  .\.venv\Scripts\python.exe .\esp32_bluetooth_command.py --port COM7 --command health
  .\.venv\Scripts\python.exe .\esp32_bluetooth_command.py --port COM7 --command "angle 90"
"""

from __future__ import annotations

import argparse
import sys
import time

import serial
from serial.tools import list_ports


def print_ports() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found. Pair ESP32-Servo in Windows Bluetooth settings first.")
        return 1
    for port in ports:
        print(f"{port.device:8} {port.description}")
    return 0


def send_command(port: str, command: str, timeout: float) -> int:
    try:
        with serial.Serial(port, 115200, timeout=0.1, write_timeout=timeout) as connection:
            connection.reset_input_buffer()
            connection.write((command.strip() + "\n").encode("utf-8"))
            connection.flush()
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                response = connection.readline().decode("utf-8", errors="replace").strip()
                if response:
                    print(response)
                    return 0 if response.startswith("OK ") else 1
            print("No response. Confirm this is the ESP32-Servo outgoing Bluetooth COM port.", file=sys.stderr)
            return 1
    except serial.SerialException as error:
        print(f"Could not open {port}: {error}", file=sys.stderr)
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a Bluetooth-serial command to ESP32-Servo.")
    parser.add_argument("--list", action="store_true", help="List COM ports, including Bluetooth serial ports.")
    parser.add_argument("--port", help="Outgoing Bluetooth COM port for ESP32-Servo, e.g. COM7.")
    parser.add_argument("--command", help='Command: "health", "help", or "angle 90".')
    parser.add_argument("--timeout", type=float, default=4.0, help="Response timeout in seconds (default: 4).")
    args = parser.parse_args()

    if args.list:
        return print_ports()
    if not args.port or not args.command:
        parser.error("use --list, or provide both --port and --command")
    if args.timeout <= 0:
        parser.error("--timeout must be positive")
    return send_command(args.port, args.command, args.timeout)


if __name__ == "__main__":
    raise SystemExit(main())
