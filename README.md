# Raspberry Pi 4 + Pi Camera: person detection (YOLO)

Files: `person_detector.py` (the program), `person-detector.service` (optional auto-start on boot).

> Not tested on real hardware. If something fails, send me the error text.

## How the ESP32s find out about a person

```
Pi camera -> YOLO (person only) -> debounce -> Blynk cloud HTTPS API (virtual pin V20 = 1/0)
                                                   |-> Door board (ESP32 #1)  -> LCD message
                                                   |-> Env board  (ESP32 #2)  -> buzzer + RGB LED
```

- The Pi writes `V20` on **both** Blynk devices (one HTTPS call each). Each ESP32 receives it like a button press (`BLYNK_WRITE(V20)`).
- It sends only on **changes**, plus a keep-alive `1` every 30 s while someone is present. The ESP32s treat "person" as cleared if no refresh arrives for 90 s, so a crashed Pi can't leave them stuck.
- Blynk's limit for update requests is 500,000 per device per day, far above what this uses. I don't know whether these calls also count toward your plan's monthly message allowance, so check your Blynk plan.
- This depends on the internet. The existing local safety logic (door lock, flame/CO alarm) does not.

## 1. Blynk setup

1. On **both** devices (door and env), add a datastream: **Virtual Pin V20**, Integer, min 0, max 1, name "Person".
2. Note the **server address** shown at the bottom right of the Blynk console (like `fra1.blynk.cloud`) and the shared project **token**.
3. Put them in `BLYNK_SERVER` and `BLYNK_TOKEN` at the top of `person_detector.py`. The same token is used for the door board, environment board, and Raspberry Pi.

## 2. Raspberry Pi setup

Use **Raspberry Pi OS 64-bit** (Bookworm or newer) on the Pi 4, with the camera ribbon connected.

```bash
sudo apt update && sudo apt full-upgrade -y
sudo apt install -y python3-picamera2 python3-flask python3-requests

# check the camera is detected (this also tells you which camera module version you have)
rpicam-hello --list-cameras
```

Create a Python environment that can also see the apt-installed `picamera2`, then install YOLO:

```bash
python3 -m venv --system-site-packages ~/person-env
source ~/person-env/bin/activate
pip install ultralytics ncnn
pip install "numpy<2"        # picamera2 from apt is built for numpy 1.x; drop this line if it causes a conflict
```

Copy `person_detector.py` to `~/pi_camera/`, edit the settings at the top (the shared Blynk token, **stream username/password**), then run:

```bash
cd ~/pi_camera
python person_detector.py
```

The first run exports the model to NCNN, which takes a few minutes. After that, you should see `[detect] PERSON present` lines when someone walks in front of the camera.

**Live stream:** on your phone (same Wi-Fi) open `http://<pi-ip>:8000/` and enter the username and password. Find the Pi's address with `hostname -I`. To watch from outside the house, use a VPN such as Tailscale. Do **not** forward port 8000 to the internet.

**Auto-start on boot (optional):** edit `person-detector.service` (replace `YOUR_USER`), then:

```bash
sudo cp person-detector.service /etc/systemd/system/
sudo systemctl enable --now person-detector
journalctl -u person-detector -f      # live log
```

## 3. Tuning

| Setting | Meaning | Try this if... |
|---|---|---|
| `IMGSZ` | YOLO input size | Too slow: 256. People too far away to detect: 416 |
| `CONF` | Confidence needed | False detections: raise to 0.6. Missed people: lower to 0.4 |
| `HITS_NEEDED` / `HIT_WINDOW` | Frames needed to confirm | Flickering: raise `HITS_NEEDED` |
| `CLEAR_AFTER_S` | Seconds before "person gone" | Alerts repeat while someone stands still: raise it |

The frame rate is shown on the stream. A few frames per second is normal on a Pi 4 and enough for this job.

## 4. ESP32 side (drop-in pattern for your ESP-NOW + Blynk sketches)

I haven't seen your current ESP-NOW/Blynk sketches (the files I have are the older MQTT versions), so these are patterns to merge. **Upload your current `.ino` files and I'll merge them properly.**

### Both boards

```cpp
#define VPIN_PERSON V20
const uint32_t PERSON_TIMEOUT_MS = 90000;   // Pi refreshes every 30 s while a person is present
bool     personPresent = false;
uint32_t personLastMs  = 0;

BLYNK_WRITE(VPIN_PERSON) {
  bool nowPresent = (param.asInt() == 1);
  personLastMs = millis();
  if (nowPresent && !personPresent) onPersonDetected();   // rising edge only
  personPresent = nowPresent;
}

// call this from loop():
void checkPersonTimeout() {
  if (personPresent && millis() - personLastMs > PERSON_TIMEOUT_MS) personPresent = false;
}
```

### Door board: `onPersonDetected()`

Wake the LCD backlight and show "Person detected" for about 3 s, using the same helpers you already use for the PIR message (they work with your DFRobot RGB LCD).

### Env board: `onPersonDetected()` and outputs

Proposed behaviour (tell me if you want something different):

- **Buzzer:** the same two short beeps as a PIR event, once when the person first appears. An active flame/CO alarm always takes priority.
- **RGB LED:** blue while `personPresent` is true (same colour as "person approaching"). Priority stays red > yellow > blue > green.
