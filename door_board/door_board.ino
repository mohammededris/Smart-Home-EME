/*
  ESP32 #1 - DOOR BOARD  (ESP-NOW + Blynk)

  Components: 16x2 I2C LCD, 4x4 keypad, MFRC522 RFID, PIR, SG90 servo
  Needs: arduino-esp32 core 3.x, and these libraries:
    Blynk, DFRobot_RGBLCD1602 (DFRobot),
    Keypad (Mark Stanley, Alexander Brevig), MFRC522 (GithubCommunity / miguelbalboa),
    ESP32Servo (Kevin Harrington, John K. Bennett)

  Behaviour (all from your answers; items marked [DEFAULT] are the proposed defaults you accepted):
    - Door unlocks with a valid RFID card OR a valid keypad PIN. Servo 0 deg = locked, 90 deg = unlocked.
    - No lockout after wrong attempts.
    - Auto-relock after RELOCK_MS                                   [DEFAULT 5 s]
    - PIR motion: LCD backlight on, message on LCD, event sent to board B (buzzer there).
    - Board B "person approaching" event: LCD backlight on + message.
    - LCD default screen: row 1 = temperature/humidity from board B, row 2 = door state.
      Alarms, PIN entry and event messages temporarily override it.   [DEFAULT]
    - Keypad: digits = PIN, 6 = enter, * = clear
    - Remote commands from the phone dashboard:
        topic home/cmd/door   payload  UNLOCK:<REMOTE_UNLOCK_CODE>  or  LOCK

  The door lock, keypad, RFID and PIR all work locally even if Wi-Fi is down.
  Networking runs in its own FreeRTOS task so a slow connection never blocks the door logic.
*/

#define BLYNK_TEMPLATE_ID "TMPL2J0ZRpAZa"
#define BLYNK_TEMPLATE_NAME "Smart Home"
#define BLYNK_AUTH_TOKEN "uTdr391bM_DM-2lwivcu773MOelFaMZL"

#include <WiFi.h>
#include <esp_now.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <DFRobot_RGBLCD1602.h>
#include <Keypad.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>

// ============================== USER SETTINGS ==============================
const char* WIFI_SSID      = "mate";
const char* WIFI_PASS      = "Hhhhnnnn15";

// Door credentials (PLACEHOLDERS - change them)
const char* VALID_PIN = "1234";
const char* VALID_UIDS[] = { "C3A0B72C", "43528A36", "92BCB502" };   // uppercase hex, no spaces. The Serial Monitor prints every scanned UID.
const size_t VALID_UID_COUNT = sizeof(VALID_UIDS) / sizeof(VALID_UIDS[0]);
const char  PIN_ENTER_KEY = '6';

// Remote unlock from the phone (set REMOTE_UNLOCK_ENABLED=false to disable remote unlock completely)
const bool  REMOTE_UNLOCK_ENABLED = true;
const char* REMOTE_UNLOCK_CODE    = "654321";

// Timing / angles
const int      ANGLE_LOCKED        = 0;
const int      ANGLE_UNLOCKED      = 90;
const uint32_t RELOCK_MS           = 5000;    // [DEFAULT] auto-relock delay
const uint32_t BACKLIGHT_MS        = 15000;   // [DEFAULT] backlight stays on this long after activity
const uint32_t PIN_TIMEOUT_MS      = 10000;   // clear a half-typed PIN after this idle time
const uint32_t PIR_WARMUP_MS       = 30000;   // ignore PIR right after boot (sensor settling)
const uint32_t PIR_COOLDOWN_MS     = 10000;   // minimum gap between two PIR events
const uint32_t ENV_STALE_MS        = 30000;   // show "--" if no data from board B for this long
const size_t   MAX_PIN_LEN         = 8;
const uint8_t  LCD_RGB_ADDR        = 0x60;
const uint8_t  LCD_ADDR            = 0x2D;

// ============================== PINS (from your wiring sheet) ==============================
#define PIN_SDA      21
#define PIN_SCL      22
#define PIN_RFID_SS   5
#define PIN_RFID_SCK 18
#define PIN_RFID_MOSI 23
#define PIN_RFID_MISO 19
#define PIN_RFID_RST 27
#define PIN_SERVO     4
#define PIN_PIR      36

// ============================== OBJECTS ==============================
DFRobot_RGBLCD1602 lcd(LCD_RGB_ADDR, LCD_ADDR);
MFRC522 rfid(PIN_RFID_SS, PIN_RFID_RST);
Servo servo;

char keys[4][4] = {
  { '1', '2', '3', 'A' },
  { '4', '5', '6', 'B' },
  { '7', '8', '9', 'C' },
  { '*', '0', '#', 'D' }
};
byte rowPins[4] = { 13, 14, 25, 26 };
byte colPins[4] = { 32, 33, 16, 17 };
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, 4, 4);

// ============================== SHARED STATE (loop <-> ESP-NOW callback) ==============================
enum PacketType : uint8_t {
  PKT_DOOR_STATE = 1, PKT_PIR, PKT_ENV_DATA, PKT_ENV_STATE,
  PKT_ENV_ALARM, PKT_ENV_SILENCED, PKT_ENV_APPROACH,
  PKT_CMD_SILENCE, PKT_CMD_DOOR
};
struct EspNowPacket {
  uint8_t type;
  uint32_t sequence;
  float temperature, humidity, distance;
  int32_t co, flame;
  bool flag;
  char text[16];
};
QueueHandle_t espNowQ;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile bool     g_cmdUnlock   = false;
volatile bool     g_cmdLock     = false;
volatile bool     g_cmdRejected = false;
volatile bool     g_approachRise = false;

// Data from board B (protected by mux)
float    g_temp = 0, g_hum = 0;
uint32_t g_lastEnvMs = 0;
char     g_envState[12] = "NORMAL";
char     g_envAlarm[16] = "NONE";
bool     g_envSilenced = false;
bool     g_approach = false;
char     g_doorState[10] = "LOCKED";

// ============================== LOOP-SIDE STATE ==============================
bool     doorUnlocked = false;
uint32_t unlockedAt = 0;

char     pinBuf[MAX_PIN_LEN + 1] = "";
size_t   pinLen = 0;
uint32_t lastKeyMs = 0;

uint32_t lastRfidMs = 0;
uint32_t lastPirEvent = 0;
bool     pirPrev = false;

uint32_t blOnUntil = 0;
bool     blIsOn = true;

char     msg1[17] = "", msg2[17] = "";
uint32_t msgUntil = 0;

char     shown1[17] = "", shown2[17] = "";
bool     alarmOnNow = false;

#define T_DOOR_STATE "home/door/state"
#define T_PIR        "home/door/pir"

// ============================== ESP-NOW ==============================
uint32_t txSequence = 0;
const uint8_t BROADCAST_MAC[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

void sendPacket(EspNowPacket p) {
  p.sequence = ++txSequence;
  esp_now_send(BROADCAST_MAC, reinterpret_cast<uint8_t*>(&p), sizeof(p));
}

void onEspNowReceive(const esp_now_recv_info_t*, const uint8_t* data, int len) {
  if (len == sizeof(EspNowPacket)) {
    EspNowPacket p;
    memcpy(&p, data, sizeof(p));
    xQueueSendFromISR(espNowQ, &p, nullptr);
  }
}

void onEspNowSent(const wifi_tx_info_t*, esp_now_send_status_t) {}

void setupEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] init failed");
    return;
  }
  esp_now_register_recv_cb(onEspNowReceive);
  esp_now_register_send_cb(onEspNowSent);
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = 0;
  peer.encrypt = false;
  esp_now_add_peer(&peer);
  Serial.println("[ESP-NOW] ready");
}

void sendTextPacket(uint8_t type, const char* text) {
  EspNowPacket p{};
  p.type = type;
  strlcpy(p.text, text, sizeof(p.text));
  sendPacket(p);
}

void pub(const char* topic, const char* payload, bool = false) {
  if (!strcmp(topic, "home/door/state")) sendTextPacket(PKT_DOOR_STATE, payload);
  else if (!strcmp(topic, "home/door/pir")) sendTextPacket(PKT_PIR, payload);
}

void wakeBacklight() { blOnUntil = millis() + BACKLIGHT_MS; }

void showMsg(const char* a, const char* b, uint32_t ms) {
  strlcpy(msg1, a, sizeof(msg1));
  strlcpy(msg2, b, sizeof(msg2));
  msgUntil = millis() + ms;
  wakeBacklight();
}

void clearPin() { pinLen = 0; pinBuf[0] = 0; }

// ============================== ESP-NOW PACKET HANDLING ==============================
void handleEnvPacket(const EspNowPacket& p) {
  portENTER_CRITICAL(&mux);
  if (p.type == PKT_ENV_DATA) {
    g_temp = p.temperature;
    g_hum = p.humidity;
    g_lastEnvMs = millis();
    if (g_lastEnvMs == 0) g_lastEnvMs = 1;
  } else if (p.type == PKT_ENV_STATE) {
    strlcpy(g_envState, p.text, sizeof(g_envState));
  } else if (p.type == PKT_ENV_ALARM) {
    strlcpy(g_envAlarm, p.text, sizeof(g_envAlarm));
  } else if (p.type == PKT_ENV_SILENCED) {
    g_envSilenced = p.flag;
  } else if (p.type == PKT_ENV_APPROACH) {
    bool a = p.flag;
    if (a && !g_approach) g_approachRise = true;
    g_approach = a;
  }
  portEXIT_CRITICAL(&mux);
}

void processEspNowPackets() {
  EspNowPacket p;
  while (xQueueReceive(espNowQ, &p, 0) == pdTRUE) {
    if (p.type == PKT_CMD_DOOR) {
      if (!strcmp(p.text, "LOCK")) g_cmdLock = true;
      else if (!strncmp(p.text, "UNLOCK:", 7)) {
        if (REMOTE_UNLOCK_ENABLED && !strcmp(p.text + 7, REMOTE_UNLOCK_CODE)) g_cmdUnlock = true;
        else g_cmdRejected = true;
      }
    } else {
      handleEnvPacket(p);
    }
  }
}

void updateBlynk() {
  static uint32_t last = 0;
  if (millis() - last < 5000) return;
  last = millis();
  float t, h;
  char st[12], al[16];
  portENTER_CRITICAL(&mux);
  t = g_temp; h = g_hum;
  strlcpy(st, g_envState, sizeof(st));
  strlcpy(al, g_envAlarm, sizeof(al));
  portEXIT_CRITICAL(&mux);
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
  Blynk.virtualWrite(V5, st);
  Blynk.virtualWrite(V6, al);
  Blynk.virtualWrite(V7, doorUnlocked ? "UNLOCKED" : "LOCKED");
}

BLYNK_WRITE(V8) {
  if (param.asInt()) sendTextPacket(PKT_CMD_SILENCE, "SILENCE");
}

BLYNK_WRITE(V9) {
  if (param.asInt()) {
    char command[24];
    snprintf(command, sizeof(command), "UNLOCK:%s", REMOTE_UNLOCK_CODE);
    sendTextPacket(PKT_CMD_DOOR, command);
  }
}

BLYNK_WRITE(V10) {
  if (param.asInt()) sendTextPacket(PKT_CMD_DOOR, "LOCK");
}

// ============================== DOOR CONTROL ==============================
void setDoor(bool unlock) {
  doorUnlocked = unlock;
  servo.write(unlock ? ANGLE_UNLOCKED : ANGLE_LOCKED);
  if (unlock) unlockedAt = millis();
  const char* s = unlock ? "UNLOCKED" : "LOCKED";
  portENTER_CRITICAL(&mux);
  strlcpy(g_doorState, s, sizeof(g_doorState));
  portEXIT_CRITICAL(&mux);
  pub(T_DOOR_STATE, s, true);
  Serial.printf("[DOOR] %s\n", s);
}

void handleAutoRelock() {
  if (doorUnlocked && millis() - unlockedAt >= RELOCK_MS) {
    setDoor(false);
    wakeBacklight();
  }
}

void handleRemoteCmds() {
  if (g_cmdUnlock) {
    g_cmdUnlock = false;
    setDoor(true);
    showMsg("Remote unlock", "Door open", 2000);
  }
  if (g_cmdLock) {
    g_cmdLock = false;
    setDoor(false);
    showMsg("Remote lock", "Door locked", 2000);
  }
  if (g_cmdRejected) {
    g_cmdRejected = false;
    showMsg("Remote unlock", "Rejected", 2000);
    Serial.println("[BLYNK] remote unlock rejected (wrong code or disabled)");
  }
}

void handleKeypad() {
  char k = keypad.getKey();
  if (k) {
    wakeBacklight();
    lastKeyMs = millis();
    if (k == PIN_ENTER_KEY) {
      if (pinLen > 0) {
        if (!strcmp(pinBuf, VALID_PIN)) {
          setDoor(true);
          showMsg("PIN accepted", "Door open", 2000);
        } else {
          showMsg("Wrong PIN", "Try again", 2000);
          Serial.println("[KEYPAD] wrong PIN");
        }
        clearPin();
      }
    } else if (k >= '0' && k <= '9') {
      if (pinLen < MAX_PIN_LEN) {
        pinBuf[pinLen++] = k;
        pinBuf[pinLen] = 0;
      }
    } else if (k == '*') {
      clearPin();
    }
  }
  if (pinLen > 0 && millis() - lastKeyMs > PIN_TIMEOUT_MS) clearPin();
}

void handleRfid() {
  static uint32_t lastPoll = 0;
  uint32_t now = millis();
  if (now - lastPoll < 100) return;
  lastPoll = now;
  if (lastRfidMs != 0 && now - lastRfidMs < 1500) return;   // avoid double reads of the same card

  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return;

  char hex[21] = "";
  byte sz = rfid.uid.size > 10 ? 10 : rfid.uid.size;
  for (byte i = 0; i < sz; i++) sprintf(hex + 2 * i, "%02X", rfid.uid.uidByte[i]);
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  lastRfidMs = now;
  if (lastRfidMs == 0) lastRfidMs = 1;

  Serial.printf("[RFID] card UID: %s\n", hex);
  wakeBacklight();

  bool ok = false;
  for (size_t i = 0; i < VALID_UID_COUNT; i++) {
    if (!strcmp(hex, VALID_UIDS[i])) { ok = true; break; }
  }
  if (ok) {
    setDoor(true);
    showMsg("Card accepted", "Door open", 2000);
  } else {
    showMsg("Card denied", "Unknown card", 2000);
  }
}

void handlePir() {
  if (millis() < PIR_WARMUP_MS) return;
  bool cur = digitalRead(PIN_PIR);
  if (cur && !pirPrev && millis() - lastPirEvent > PIR_COOLDOWN_MS) {
    lastPirEvent = millis();
    wakeBacklight();
    showMsg("Motion", "detected", 3000);
    pub(T_PIR, "1", false);          // board B sounds the buzzer
    Serial.println("[PIR] motion");
  }
  pirPrev = cur;
}

void handleApproach() {
  if (g_approachRise) {
    g_approachRise = false;
    showMsg("Person", "approaching", 3000);
    Serial.println("[ENV] person approaching");
  }
}

// ============================== LCD ==============================
void renderLcd() {
  char st[12], al[16];
  bool silenced;
  float t, h;
  uint32_t last;
  portENTER_CRITICAL(&mux);
  strlcpy(st, g_envState, sizeof(st));
  strlcpy(al, g_envAlarm, sizeof(al));
  silenced = g_envSilenced;
  t = g_temp; h = g_hum; last = g_lastEnvMs;
  portEXIT_CRITICAL(&mux);

  alarmOnNow = strcmp(al, "NONE") != 0;
  bool warnOn = !alarmOnNow && !strcmp(st, "WARNING");

  char l1[24], l2[24];
  if (alarmOnNow) {
    if (!strcmp(al, "FLAME"))    strcpy(l1, "!!FLAME ALARM!!");
    else if (!strcmp(al, "CO"))  strcpy(l1, "!! CO ALARM !!");
    else                         strcpy(l1, "!!FLAME + CO !!");
    strcpy(l2, silenced ? "Buzzer silenced" : "Check the house");
  } else if (millis() < msgUntil) {
    strlcpy(l1, msg1, sizeof(l1));
    strlcpy(l2, msg2, sizeof(l2));
  } else if (pinLen > 0) {
    strcpy(l1, "Enter PIN:");
    size_t i = 0;
    for (; i < pinLen && i < 16; i++) l2[i] = '*';
    l2[i] = 0;
  } else if (warnOn) {
    strcpy(l1, "CO WARNING");
    strcpy(l2, "CO level high");
  } else {
    bool fresh = last != 0 && (millis() - last) < ENV_STALE_MS;
    if (fresh) snprintf(l1, sizeof(l1), "T:%.0fC H:%.0f%%", t, h);
    else       strcpy(l1, "T:--C H:--%");
    strcpy(l2, doorUnlocked ? "UNLOCKED" : "LOCKED");
  }

  char w1[17], w2[17];
  snprintf(w1, sizeof(w1), "%-16.16s", l1);
  snprintf(w2, sizeof(w2), "%-16.16s", l2);
  if (strcmp(w1, shown1)) { lcd.setCursor(0, 0); lcd.print(w1); strcpy(shown1, w1); }
  if (strcmp(w2, shown2)) { lcd.setCursor(0, 1); lcd.print(w2); strcpy(shown2, w2); }
}

void updateBacklight() {
  bool want = (millis() < blOnUntil) || alarmOnNow;
  if (want != blIsOn) {
    blIsOn = want;
    if (want) lcd.setRGB(255, 255, 255); else lcd.setRGB(0, 0, 0);
  }
}

// ============================== SETUP / LOOP ==============================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] Door board");

  espNowQ = xQueueCreate(16, sizeof(EspNowPacket));
  setupEspNow();
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(5000);

  Wire.begin(PIN_SDA, PIN_SCL);
  lcd.init();
  lcd.setRGB(255, 255, 255);
  lcd.setCursor(0, 0);
  lcd.print("Door board      ");
  lcd.setCursor(0, 1);
  lcd.print("Starting...     ");
  wakeBacklight();

  servo.setPeriodHertz(50);
  servo.attach(PIN_SERVO, 500, 2400);
  servo.write(ANGLE_LOCKED);

  SPI.begin(PIN_RFID_SCK, PIN_RFID_MISO, PIN_RFID_MOSI, PIN_RFID_SS);
  rfid.PCD_Init();
  rfid.PCD_DumpVersionToSerial();

  pinMode(PIN_PIR, INPUT);

  keypad.setDebounceTime(30);

}

void loop() {
  Blynk.run();
  processEspNowPackets();
  updateBlynk();
  handleRemoteCmds();
  handleKeypad();
  handleRfid();
  handlePir();
  handleApproach();
  handleAutoRelock();
  renderLcd();
  updateBacklight();
  delay(5);
}
