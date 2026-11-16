# ESP32 servo-driver board — firmware backup

Full-chip image taken **2026-09-18**, before the board was reflashed with the
ST3215 test firmware.

| | |
|---|---|
| File | `esp32_stock_full_4MB_30c922b03c34.bin` |
| SHA-256 | `31769bea830af690c5744700dee72a1d7f26c268f9e7d02a7a54a57c0fb9595d` |
| Board | Waveshare ESP32 Servo Driver Expansion Board |
| Chip | ESP32-D0WD-V3, revision v3.1, 4 MB flash, 40 MHz crystal |
| MAC | `30:c9:22:b0:3c:34` |
| Contents | Waveshare **ServoDriver** demo: web UI with Normal / Leader / Follower modes, per-servo position, load, voltage, temperature, and "ID to Set". Built with Arduino-ESP32 core 3.3.2 on a Windows PC |

Verified after reading: valid image header at 0x1000, intact partition table
(nvs, otadata, app0, app1, spiffs, coredump), ~1 MB of non-blank code.

## Restore

This writes back the *entire* chip — bootloader, partition table, app, and the
NVS/SPIFFS data (saved Wi-Fi settings included) — exactly as it was.

```bash
sg dialout -c "~/.platformio/penv/bin/python \
  ~/.platformio/packages/tool-esptoolpy/esptool.py \
  --port /dev/ttyUSB0 --baud 460800 \
  write_flash 0x0 esp32_stock_full_4MB_30c922b03c34.bin"
```

Check the file first: `sha256sum esp32_stock_full_4MB_30c922b03c34.bin` must
match the hash above.

Keep this file. It is the only copy of that build; the source it came from is on
the Windows machine, not in this workspace.
