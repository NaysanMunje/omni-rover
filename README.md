# Omni Rover

Three-wheel kiwi-drive rover on a [Freenove ESP32-S3-WROOM](https://github.com/Freenove/Freenove_ESP32_S3_WROOM_Board) with a phone web controller.

Current firmware: [`three_motor_test/three_motor_test.ino`](three_motor_test/three_motor_test.ino)

Controller: join the same Wi-Fi as the rover, then open `http://<board-ip>` or `http://omnirover.local`. Use **http**, not https.

## Hardware

| Piece | Role |
| --- | --- |
| Freenove ESP32-S3-WROOM | MCU, Wi-Fi, web UI |
| TB6612FNG | Rear motors M1 (left) and M2 (right) |
| L298N | Front motor M3 |
| 3× 60 mm omni wheels | Equilateral ~220 mm spacing |
| Encoders | Yellow = A, green = B, blue = 3.3 V, black = GND |
| 12 V supply | Motor power. Common GND with ESP32, TB6612, and L298N |

Kiwi layout:

- **M3 L298N** — front, wheel axis 90°
- **M1 TB6612 A** — left rear, 210°
- **M2 TB6612 B** — right rear, −30°

The camera module is **not** used. Camera GPIOs are free for motors. Encoder B on M2 uses GPIO 37 (octal PSRAM), so firmware is flashed with **PSRAM disabled**.

## Wiring

Motor windings: red / white. Common GND: 12 V−, ESP32, TB6612, L298N.

**TB6612**

- VM → 12 V+, VCC → ESP32 3.3 V, GND → common
- STBY → GPIO 3 (driven high)
- M1: red → AO1, white → AO2, PWMA → 48, AIN1 → 41, AIN2 → 42, enc A/B → 1 / 2
- M2: red → BO1, white → BO2, PWMB → 18, BIN1 → 40, BIN2 → 39, enc A/B → 38 / 37

**L298N**

- 12 V+ → +12 V / VMS, GND → common
- Remove the ENA jumper so PWM can control speed
- Logic 5 V from the L298N jumper or ESP32 5 V
- M3: red → OUT1, white → OUT2, ENA → 15, IN1 → 47, IN2 → 21, enc A/B → 13 / 14

Unused on purpose: PSRAM 35–36, USB 19–20, UART 43–44, BOOT 0, straps 45–46.

## Firmware features

- Home Wi-Fi station mode (not an access point)
- **Test** — spin all motors, omni forward/back, speed sliders, per-motor invert
- **Controller** — hold-to-drive pad with diagonals and spin; slide between buttons without lifting
- **Joystick** — left stick holonomic drive, right stick yaw; both at once
- **Testing** — three left/right mixes (default A: rear 50% / front 100%)
- 350 ms PWM coast ramp on all three motors when stopping

Body mix (vx right, vy forward, wz CCW):

```
s1 = k * vx - 0.866 * vy + wz   // M1
s2 = k * vx + 0.866 * vy + wz   // M2
s3 = -vx + wz                   // M3
```

`k` is 0.5 (A), 0.866 (B), or 0.32 (C).

## Setup and flash

1. Install [Arduino CLI](https://arduino.github.io/arduino-cli/) and the ESP32 board package.
2. Copy Wi-Fi credentials:

```text
copy three_motor_test\secrets.example.h three_motor_test\secrets.h
```

Edit `secrets.h` with your SSID and password. That file is gitignored.

3. Board: **ESP32S3 Dev Module**. UART USB (CH343) is typically **COM6** on Windows. Close Serial Monitor before upload.
4. Compile and upload with PSRAM **disabled** (GPIO 37 is an encoder):

```powershell
arduino-cli compile --fqbn "esp32:esp32:esp32s3:PSRAM=disabled,FlashSize=8M,PartitionScheme=default_8MB,USBMode=hwcdc,UploadMode=default,CDCOnBoot=default,CPUFreq=240,FlashMode=qio,UploadSpeed=921600,DebugLevel=none" three_motor_test
arduino-cli upload -p COM6 --fqbn "esp32:esp32:esp32s3:PSRAM=disabled,FlashSize=8M,PartitionScheme=default_8MB,USBMode=hwcdc,UploadMode=default,CDCOnBoot=default,CPUFreq=240,FlashMode=qio,UploadSpeed=921600,DebugLevel=none" three_motor_test
```

5. Serial 115200 prints `http://x.x.x.x` when Wi-Fi connects. Phone and rover must be on the same network (use 2.4 GHz if the phone’s 5 GHz band cannot see the board).

## Other sketches

Bring-up helpers, not the main rover UI:

| Sketch | Purpose |
| --- | --- |
| `gem_test/` | Single TB6612 motor + encoder (known-good driver pattern) |
| `l298n_test/` | Single L298N motor + encoder |
| `camera_ap_stream/` | Optional camera AP + MJPEG (camera not fitted on the current bot) |
| `motor_min_test/`, `motor_debug/`, `motor_cw_ccw/`, `gpio_drive_scan/` | Early GPIO / motor debug |

## License

MIT. See [LICENSE](LICENSE).
