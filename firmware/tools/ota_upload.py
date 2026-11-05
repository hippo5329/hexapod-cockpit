#!/usr/bin/env python3
"""Send a firmware image to the miniHexa over Wi-Fi (ArduinoOTA, port 3232).

    ota_upload.py <board IP> <firmware.bin> <password file> [espota.py]

The password is read from the file (its first line), never from the command line.
espota.py is the Arduino core's uploader; by default the one in PlatformIO's
framework-arduinoespressif32 package. The board must reach this host on the TCP port
espota listens on (it connects back to fetch the image): run it on the board's LAN, not
behind NAT.
"""
import importlib.util
import inspect
import logging
import os
import random
import sys


def upload(host, image, pw_file, espota_path=None, settings=False):
    """Sends image with espota: a firmware update, or (settings) the SPIFFS command that
    push_settings.py uses. The return code is espota's (0: done)."""
    espota_path = espota_path or os.path.join(
        os.environ.get("PLATFORMIO_CORE_DIR") or os.path.expanduser("~/.platformio"),
        "packages", "framework-arduinoespressif32", "tools", "espota.py")
    with open(pw_file) as f:
        password = f.readline().strip()
    if not password:
        sys.exit("empty password file")
    spec = importlib.util.spec_from_file_location("espota", espota_path)
    espota = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(espota)
    # espota sets these from its own command line in main(); serve() reads them as globals.
    espota.TIMEOUT = 10
    espota.PROGRESS = False
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
    local_port = random.randint(10000, 60000)
    command = espota.SPIFFS if settings else espota.FLASH
    # Newer espota.py takes an md5_target argument before the file name; older ones do not.
    if "md5_target" in inspect.signature(espota.serve).parameters:
        return espota.serve(host, "0.0.0.0", 3232, local_port, password, False, image, command)
    return espota.serve(host, "0.0.0.0", 3232, local_port, password, image, command)


def main():
    if len(sys.argv) not in (4, 5):
        sys.exit(__doc__)
    host, image, pw_file = sys.argv[1:4]
    rc = upload(host, image, pw_file, sys.argv[4] if len(sys.argv) == 5 else None)
    print(f"OTA {'done' if rc == 0 else 'FAILED'} (rc {rc})")
    sys.exit(rc)


if __name__ == "__main__":
    main()
