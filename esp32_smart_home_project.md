# ESP32 Smart Home Project (ESP-NOW + Blynk)

The two ESP32 boards talk directly using **ESP-NOW**. Each board also connects to the same Wi-Fi network for its own **Blynk IoT** dashboard connection. There is no broker and no UART wire.

Files delivered with this document:

| File | Board |
|---|---|
| `door_board/door_board.ino` | ESP32 #1 (Door / Access) |
| `env_board/env_board.ino` | ESP32 #2 (Environment / Safety) |

> The sketches use the ESP32 Arduino core 3.x ESP-NOW callback API. Install the Blynk library and configure the Wi-Fi and shared Blynk project token before compiling.

---

## 1. Decisions

| Topic | Decision |
|---|---|
| Link between boards | ESP-NOW broadcast packets, no broker |
| Phone dashboard | Blynk IoT |
| Unlock logic | **RFID card OR keypad PIN** |
| LCD | DFRobot 1602RGB I2C display (LCD address 0x2D, RGB address 0x60) |
| Keypad | **4x4** |
| PIN / card UIDs | **Placeholders** in the code (PIN `1234`, UIDs `DEADBEEF`, `01020304`) |
| Servo | **0° locked, 90° unlocked** |
| Auto-relock | **5 s** (proposed default, accepted) |
| Lockout | **None** |
| PIR | Wake LCD backlight, show message, **buzzer on board B** |
| Ultrasonic | HC-SR04, detects **person approaching**, trigger **< 100 cm** (default) |
| DHT11 | Display only |
| Gas sensor | **Gravity analog CO sensor (MQ-7)**, analog, no heater cycle |
| Flame sensor | **Analog** |
| Buzzer | **Passive** (PWM tone) |
| RGB (HW-478) | Green normal, Blue approach/motion, Yellow CO warning, Red flame/CO alarm |
| Alarm behavior | Continuous buzzer, **stops by itself when the condition clears**; can be silenced from the phone |
| LCD default screen | Row 1: temperature + humidity, Row 2: door status |
| Keypad keys | digits = PIN, `#` = enter, `*` = clear |

---

## 2. Wiring (from your spreadsheet, unchanged)

### ESP32 #1 (Door)

| Component | Module pin | ESP32 pin | Power |
|---|---|---|---|
| LCD I2C | SDA / SCL | GPIO21 / GPIO22 | 5V (level shifter recommended) |
| Keypad rows | R1 R2 R3 R4 | GPIO13, 14, 25, 26 | none |
| Keypad columns | C1 C2 C3 C4 | GPIO32, 33, 16, 17 | none |
| RFID RC522 | SDA(SS) / SCK / MOSI / MISO / RST | GPIO5 / 18 / 23 / 19 / 27 | **3.3V only** |
| SG90 servo | Signal | GPIO4 | 5V, separate supply + 470 µF cap, common GND |
| PIR | OUT | GPIO36 (VP) | 5V |

### ESP32 #2 (Environment)

| Component | Module pin | ESP32 pin | Power |
|---|---|---|---|
| DHT11 | DATA | GPIO4 | 3.3V |
| CO sensor (code assumes the sheet's gas pin) | AO through divider | GPIO34 | 5V |
| Flame sensor | **AO** (code assumes analog) | GPIO39 (VN) | 3.3V |
| HC-SR04 | TRIG | GPIO26 | 5V |
| | ECHO through divider | GPIO27 | |
| HW-478 RGB | R / G / B | GPIO16 / 17 / 32 | GND (common cathode) |
| Passive buzzer | signal | GPIO25 | 3.3V |

Not used by the code: the **photoresistor** (GPIO35) and the **2-channel relay** (GPIO13/14) from your sheet, because they were not in your component list. Tell me if you want them.

**Voltage divider** (for HC-SR04 ECHO and the CO sensor AO): your 1k/2k gives 3.33 V, which your own sheet marks as slightly too high. Use **2 kΩ in series + 3.3 kΩ to GND** (about 3.1 V). Check your Gravity CO module's real output range first.

Notes: no UART pins are needed any more. Both boards are Wi-Fi only.

---

## 3. Wi-Fi, ESP-NOW, and Blynk setup

1. Create two Blynk devices from your template: one for the door board and one for the environment board.
2. Put the same Wi-Fi name/password in both sketches.
3. Put the same shared Blynk project token in both sketches:
   - `door_board/door_board.ino`
   - `env_board/env_board.ino`
4. Both boards must use the same Wi-Fi access point. ESP-NOW uses the access point's Wi-Fi channel while Blynk uses the normal Wi-Fi connection.
5. Install `Blynk`, plus the sensor/display libraries listed below. No MQTT library or broker is needed.

Free tier notes (from third-party pages, check HiveMQ's pricing page for current numbers): roughly 100 connections and 10 GB/month. Your traffic is tiny: about 5 readings every 5 s per board.

---

## 4. Arduino setup

- Board package: **arduino-esp32 core 3.x** (the code uses the 3.x `ledcAttach` / `ledcWriteTone` API).
- Libraries:
  - Door board: `Blynk`, `DFRobot_RGBLCD1602`, `Keypad`, `MFRC522`, `ESP32Servo`
  - Environment board: `Blynk`, `DHT sensor library` (Adafruit) + `Adafruit Unified Sensor`

---

## 5. ESP-NOW packets and Blynk dashboard

The boards exchange typed ESP-NOW packets directly; no MQTT broker is required. Both boards must connect to the same Wi-Fi access point because ESP-NOW uses that access point's channel.

Create two Blynk devices in the same project and put the shared project token in both sketches. The current virtual-pin assignments are:

| Virtual pin | Environment board | Door board |
|---|---|---|
| `V0` | Temperature | Environment temperature |
| `V1` | Humidity | Environment humidity |
| `V2` | CO raw ADC | - |
| `V3` | Flame raw ADC | - |
| `V4` | Distance | - |
| `V5` | Environment state | Environment state |
| `V6` | Alarm text | Alarm text |
| `V7` | Approach | Door state |
| `V8` | Silenced state | Silence button |
| `V9` | Silence button | Unlock button |
| `V10` | - | Lock button |

The Blynk buttons generate ESP-NOW commands, so they work without a broker.

The following legacy topic names describe the values represented by the Blynk datastreams; they are not published by the ESP-NOW sketches.

### Legacy board-B value names

| Topic | Payload | Notes |
|---|---|---|
| `home/env/temperature` | `24.0` | °C, every 5 s |
| `home/env/humidity` | `45.0` | %, every 5 s |
| `home/env/co` | `850` | raw ADC 0 to 4095, every 5 s |
| `home/env/flame` | `3200` | raw ADC, every 5 s |
| `home/env/distance` | `120` | cm, `-1` = no echo, every 5 s |
| `home/env/state` | `NORMAL` / `WARNING` / `ALARM` | on change |
| `home/env/alarm` | `NONE` / `FLAME` / `CO` / `FLAME+CO` | on change |
| `home/env/silenced` | `0` / `1` | on change |
| `home/env/approach` | `0` / `1` | on change |
| `home/env/status` | `online` / `offline` | last-will message |

### Published by board A

| Topic | Payload | Notes |
|---|---|---|
| `home/door/state` | `LOCKED` / `UNLOCKED` | retained, on change |
| `home/door/pir` | `1` | one message per motion event, not retained |
| `home/door/status` | `online` / `offline` | last-will message |

### Commands (phone to boards). Always publish them with **retain OFF**

| Topic | Payload | Effect |
|---|---|---|
| `home/cmd/door` | `UNLOCK:654321` | Unlocks the door (code must match `REMOTE_UNLOCK_CODE`), then auto-relocks |
| `home/cmd/door` | `LOCK` | Locks immediately |
| `home/cmd/silence` | `SILENCE` | Silences the buzzer until the alarm condition clears |

### Who listens to what

- Board A subscribes to: `home/env/#`, `home/cmd/door`
- Board B subscribes to: `home/door/pir`, `home/cmd/silence`

---

## 6. Legacy MQTT dashboard (not used)

Do not configure an MQTT dashboard for the current sketches. Use the Blynk virtual pins in section 5 instead.

- Text/gauge widgets for `home/env/temperature`, `humidity`, `co`, `distance`, `state`, `alarm`, and `home/door/state`
- A button publishing `UNLOCK:654321` to `home/cmd/door` (retain off)
- A button publishing `LOCK` to `home/cmd/door` (retain off)
- A button publishing `SILENCE` to `home/cmd/silence` (retain off)

---

## 7. What each board does

### ESP32 #1 (Door)

| Function | Description |
|---|---|
| `handleKeypad()` | Builds the PIN from digits; `#` checks it, `*` clears. Wrong PIN shows "Wrong PIN", no lockout. Half-typed PIN clears after 10 s |
| `handleRfid()` | Reads the card UID, prints it on Serial (use that to fill in your real UIDs), checks it against `VALID_UIDS` |
| `setDoor()` | Moves the servo (0° / 90°), publishes `home/door/state` |
| `handleAutoRelock()` | Locks again 5 s after unlocking |
| `handlePir()` | Rising edge only (after 30 s boot warm-up, 10 s cooldown): wake backlight, show "Motion detected", publish `home/door/pir` |
| `handleApproach()` | When B reports a person approaching: wake backlight, show "Person approaching" |
| `handleRemoteCmds()` | Executes `UNLOCK:<code>` / `LOCK` from the phone |
| `renderLcd()` | Priority: alarm > timed message > PIN entry > CO warning > default screen |
| `updateBacklight()` | Backlight off after 15 s idle, always on during an alarm |
| `mqttTask()` | Runs on core 0: Wi-Fi + MQTT reconnect, publish queue, subscriptions |

### ESP32 #2 (Environment)

| Function | Description |
|---|---|
| DHT read | Every 2 s, display only |
| `readAdcAvg()` | Averages 16 ADC samples for CO and flame |
| `readDistanceCm()` / `updateApproach()` | HC-SR04; needs 2 near readings to set "approach", about 1 s of clear readings (and 20 cm extra margin) to clear |
| `evaluateAlarms()` | Flame and CO thresholds with hysteresis; CO ignored for the first 60 s after boot (sensor warm-up) |
| `updateRgb()` | Red > Yellow > Blue > Green |
| `updateBuzzer()` | Continuous tone on alarm (unless silenced); two short beeps on a PIR event |
| `handleEvents()` | Applies `SILENCE` and PIR events |
| `publishChanges()` / `publishReadings()` | State changes immediately, readings every 5 s |
| `mqttTask()` | Same structure as board A |

Both boards keep their local logic running when Wi-Fi or the broker is down. Only the cross-board messages need the cloud.

---

## 8. Things you must tune or know

1. **Alarm thresholds** (`CO_WARN_ADC`, `CO_ALARM_ADC`, `FLAME_THRESHOLD`) are placeholders. Open the Serial Monitor on board B (115200 baud): it prints CO and flame readings every second. Note the normal values, then the values near a lighter flame (from a safe distance) and set the thresholds between them. If your flame sensor reads *higher* with a flame, set `FLAME_TRIGGER_BELOW = false`.
2. **This is not a certified CO or fire alarm.** Don't rely on it for life safety. Use a proper CO/smoke detector as well.
3. **TLS is not fully verified.** The sketches use `setInsecure()`: traffic is encrypted but the server certificate is not checked. For a door lock, load HiveMQ's root CA with `setCACert()` once the system works.
4. **Remote unlock safety.** Anyone with your MQTT credentials can try to unlock the door, and there is no lockout. Keep the unlock code secret, use a strong MQTT password, don't leave the code saved on a shared phone, and set `REMOTE_UNLOCK_ENABLED = false` if you don't want remote unlock at all.
5. **Retained-command protection.** If a command is ever published with retain on, board A and B wipe it and ignore anything arriving in the first 3 s after (re)connecting, so an old command can't replay.
6. **PIR** needs about 30 s after power-up to settle (the code ignores it for that time).
7. **Servo** power: use a separate 5 V supply with a shared GND, or the ESP32 may reset when the servo moves.
8. **Added by me beyond your answers** (change or remove if you disagree): CO warm-up ignore time (60 s), PIR boot warm-up and cooldown, backlight timeout (15 s), PIN-entry timeout (10 s), the `UNLOCK:<code>` format, and wiping retained commands.
