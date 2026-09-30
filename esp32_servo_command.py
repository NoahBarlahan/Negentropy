r"""Send a local Wi-Fi command to the ESP32ServoWiFi sketch.

Examples (PowerShell):
  # Test the ESP32 connection; works before a servo pin is chosen.
  .\.venv\Scripts\python.exe .\esp32_servo_command.py --host 192.168.1.50 --health

  # Move after SERVO_PIN is set and the servo is wired.
  .\.venv\Scripts\python.exe .\esp32_servo_command.py --host 192.168.1.50 --angle 90
"""

from __future__ import annotations

import argparse
import json
import sys
from urllib.error import HTTPError, URLError
from urllib.request import urlopen


def request(host: str, path: str, timeout: float) -> tuple[int, str]:
    """Request an ESP32 endpoint using only Python's standard library."""
    address = host.strip().removeprefix("http://").removeprefix("https://").rstrip("/")
    with urlopen(f"http://{address}{path}", timeout=timeout) as response:
        return response.status, response.read().decode("utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Control ESP32ServoWiFi over the local network.")
    parser.add_argument("--host", required=True, help="ESP32 IP address printed by the Serial Monitor, e.g. 192.168.1.50")
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--health", action="store_true", help="Check Wi-Fi connection and firmware status.")
    action.add_argument("--angle", type=int, metavar="0-180", help="Servo target angle (requires a configured servo pin).")
    parser.add_argument("--timeout", type=float, default=3.0, help="Network timeout in seconds (default: 3).")
    args = parser.parse_args()

    if args.angle is not None and not 0 <= args.angle <= 180:
        parser.error("--angle must be from 0 through 180")

    endpoint = "/health" if args.health else f"/servo?angle={args.angle}"
    try:
        status, body = request(args.host, endpoint, args.timeout)
        try:
            print(json.dumps(json.loads(body), indent=2))
        except json.JSONDecodeError:
            print(body)
        return 0 if 200 <= status < 300 else 1
    except HTTPError as error:
        message = error.read().decode("utf-8", errors="replace")
        print(f"ESP32 returned HTTP {error.code}: {message}", file=sys.stderr)
        return 1
    except URLError as error:
        print(f"Could not reach the ESP32 at {args.host}: {error.reason}", file=sys.stderr)
        print("Check that the computer and ESP32 are on the same Wi-Fi network and use the IP shown in Serial Monitor.", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
