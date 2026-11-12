/*
  ESP32 #2 - ENVIRONMENT BOARD  (ESP-NOW + Blynk)

  Components: DHT11, flame sensor (analog), Gravity analog CO sensor (MQ-7, analog),
              HC-SR04 ultrasonic, HW-478 RGB LED, passive buzzer
  Needs: arduino-esp32 core 3.x, and these libraries:
    DHT sensor library (Adafruit) + Adafruit Unified Sensor

  Behaviour (items marked [DEFAULT] are the proposed defaults you accepted):
    - Reads DHT11 (display only), CO, flame, distance.
    - Flame or high CO  -> ALARM: continuous buzzer + RED LED. Stops by itself when the condition clears.
    - Buzzer can be silenced from the phone (topic home/cmd/silence, payload SILENCE);
      it re-arms automatically once the condition has cleared.
    - CO between warn and alarm level -> WARNING: YELLOW LED, no buzzer.
    - Person within APPROACH_CM -> BLUE LED, "approach" published for board A's LCD.   [DEFAULT 100 cm]
    - PIR event from board A -> two short beeps + BLUE LED for 3 s.                   [DEFAULT]
    - Normal -> GREEN LED.
    - Colour priority: red > yellow > blue > green.

  Alarms, buzzer and LED work locally even if Wi-Fi is down.
  Networking runs in its own FreeRTOS task so a slow connection never blocks the alarm logic.
*/

#define BLYNK_TEMPLATE_ID "TMPL2J0ZRpAZa"
#define BLYNK_TEMPLATE_NAME "Smart Home"
#define BLYNK_AUTH_TOKEN "coNSJC4SjDe5jC92ql5DcaQYCr9DPMwW"

#include <WiFi.h>
#include <esp_now.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>

// ============================== USER SETTINGS ==============================
const char* WIFI_SSID      = "mate";
const char* WIFI_PASS      = "Hhhhnnnn15";

// ---- Alarm thresholds: raw ADC values 0..4095. PLACEHOLDERS - calibrate from the Serial Monitor. ----
// CO sensor: reading goes UP with more CO. (The 2k/3.3k divider scales the sensor voltage down.)
const int      CO_WARN_ADC    = 1200;
const int      CO_ALARM_ADC   = 2000;
const int      CO_HYST        = 100;
const uint32_t CO_WARMUP_MS   = 60000;   // CO alarms are ignored this long after boot (set 0 to disable)

// Flame sensor (analog): most modules read LOWER when a flame is near. Flip FLAME_TRIGGER_BELOW if yours is the opposite.
const bool FLAME_TRIGGER_BELOW = true;
const int  FLAME_THRESHOLD     = 1500;
const int  FLAME_HYST          = 150;

// Ultrasonic "person approaching"
const float APPROACH_CM           = 100.0;   // [DEFAULT]
const float APPROACH_CLEAR_MARGIN = 20.0;    // must be this much farther than APPROACH_CM to clear

// Outputs
const bool RGB_COMMON_ANODE = false;         // HW-478 module wired with GND = common cathode (per your sheet)
const int  ALARM_TONE_HZ    = 2500;
const int  BEEP_TONE_HZ     = 2000;

const uint32_t PUBLISH_INTERVAL_MS = 5000;

// ============================== PINS (from your wiring sheet) ==============================
#define PIN_DHT     4
#define PIN_CO      34    // input-only ADC1, through the 2k/3.3k divider
#define PIN_FLAME   39    // input-only ADC1, flame sensor AO
#define PIN_TRIG    26
#define PIN_ECHO    27    // through the 2k/3.3k divider
#define PIN_LED_R   16
#define PIN_LED_G   17
#define PIN_LED_B   32
#define PIN_BUZZER  25

// ============================== OBJECTS ==============================
DHT dht(PIN_DHT, DHT11);
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

volatile bool     g_cmdSilence = false;
volatile bool     g_pirEvent = false;
volatile bool     g_fullPublish = true;

// ============================== STATE ==============================
float    temperatureC = NAN, humidityPct = NAN;
int      coRaw = 0, flameRaw = 0;
float    distanceCm = -1;

bool     flameActive = false, coWarn = false, coAlarm = false;
bool     approachNear = false;
bool     silenced = false;

enum State { ST_NORMAL, ST_WARNING, ST_ALARM };
State    state = ST_NORMAL;
char     alarmText[12] = "NONE";

// buzzer / led effects
bool     beepActive = false;
uint32_t beepStart = 0;
uint32_t pirBlueUntil = 0;
int      currentTone = 0;

// last published values (for publish-on-change)
State    pubState = ST_NORMAL;
char     pubAlarm[12] = "";
bool     pubSilenced = false, pubApproach = false;
bool     havePub = false;

#define T_STATE     "home/env/state"
#define T_ALARM     "home/env/alarm"
#define T_SILENCED  "home/env/silenced"
#define T_APPROACH  "home/env/approach"

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
  EspNowPacket p{};
  if (!strcmp(topic, "home/env/state")) {
    p.type = PKT_ENV_STATE;
    strlcpy(p.text, payload, sizeof(p.text));
  } else if (!strcmp(topic, "home/env/alarm")) {
    p.type = PKT_ENV_ALARM;
    strlcpy(p.text, payload, sizeof(p.text));
  } else if (!strcmp(topic, "home/env/silenced")) {
    p.type = PKT_ENV_SILENCED;
    p.flag = payload[0] == '1';
  } else if (!strcmp(topic, "home/env/approach")) {
    p.type = PKT_ENV_APPROACH;
    p.flag = payload[0] == '1';
  } else {
    return;
  }
  sendPacket(p);
}

const char* stateName(State s) {
  return s == ST_ALARM ? "ALARM" : (s == ST_WARNING ? "WARNING" : "NORMAL");
}

void processEspNowPackets() {
  EspNowPacket p;
  while (xQueueReceive(espNowQ, &p, 0) == pdTRUE) {
    if (p.type == PKT_PIR) g_pirEvent = true;
    else if (p.type == PKT_CMD_SILENCE && !strcmp(p.text, "SILENCE")) g_cmdSilence = true;
  }
}

// ============================== SENSORS ==============================
int readAdcAvg(int pin) {
  uint32_t sum = 0;
  for (int i = 0; i < 16; i++) {
    sum += analogRead(pin);
    delayMicroseconds(100);
  }
  return (int)(sum / 16);
}

float readDistanceCm() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long d = pulseIn(PIN_ECHO, HIGH, 25000);   // 25 ms timeout, about 4 m
  if (d == 0) return -1;
  return d * 0.0343f / 2.0f;
}

void updateApproach() {
  static int nearCount = 0, farCount = 0;
  bool isNear = (distanceCm > 0 && distanceCm < APPROACH_CM);
  bool isFar  = (distanceCm < 0 || distanceCm > APPROACH_CM + APPROACH_CLEAR_MARGIN);
  if (isNear) { nearCount++; farCount = 0; }
  else if (isFar) { farCount++; nearCount = 0; }
  if (!approachNear && nearCount >= 2) approachNear = true;     // 2 readings in a row
  if (approachNear && farCount >= 5)  approachNear = false;     // about 1 s of clear readings
}

void evaluateAlarms() {
  // Flame (with hysteresis)
  if (FLAME_TRIGGER_BELOW) {
    if (!flameActive && flameRaw < FLAME_THRESHOLD) flameActive = true;
    else if (flameActive && flameRaw > FLAME_THRESHOLD + FLAME_HYST) flameActive = false;
  } else {
    if (!flameActive && flameRaw > FLAME_THRESHOLD) flameActive = true;
    else if (flameActive && flameRaw < FLAME_THRESHOLD - FLAME_HYST) flameActive = false;
  }

  // CO (with hysteresis and warm-up)
  if (millis() < CO_WARMUP_MS) {
    coWarn = false;
    coAlarm = false;
  } else {
    if (!coAlarm && coRaw >= CO_ALARM_ADC) coAlarm = true;
    else if (coAlarm && coRaw < CO_ALARM_ADC - CO_HYST) coAlarm = false;
    if (!coWarn && coRaw >= CO_WARN_ADC) coWarn = true;
    else if (coWarn && coRaw < CO_WARN_ADC - CO_HYST) coWarn = false;
  }

  bool alarm = flameActive || coAlarm;
  if (alarm) state = ST_ALARM;
  else if (coWarn) state = ST_WARNING;
  else state = ST_NORMAL;

  if (flameActive && coAlarm) strcpy(alarmText, "FLAME+CO");
  else if (flameActive)       strcpy(alarmText, "FLAME");
  else if (coAlarm)           strcpy(alarmText, "CO");
  else                        strcpy(alarmText, "NONE");

  if (!alarm) silenced = false;   // silence only lasts until the condition clears
}

// ============================== OUTPUTS ==============================
void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  if (RGB_COMMON_ANODE) { r = 255 - r; g = 255 - g; b = 255 - b; }
  ledcWrite(PIN_LED_R, r);
  ledcWrite(PIN_LED_G, g);
  ledcWrite(PIN_LED_B, b);
}

void updateRgb() {
  if (state == ST_ALARM)                              setRgb(255, 0, 0);     // red
  else if (state == ST_WARNING)                       setRgb(255, 160, 0);   // yellow/amber
  else if (approachNear || millis() < pirBlueUntil)   setRgb(0, 0, 255);     // blue
  else                                                setRgb(0, 255, 0);     // green
}

void updateBuzzer() {
  int desired = 0;
  if (state == ST_ALARM && !silenced) {
    desired = ALARM_TONE_HZ;                 // continuous alarm
  } else if (beepActive) {
    uint32_t t = millis() - beepStart;       // two short beeps: 0-150 on, 150-250 off, 250-400 on
    if (t < 150) desired = BEEP_TONE_HZ;
    else if (t < 250) desired = 0;
    else if (t < 400) desired = BEEP_TONE_HZ;
    else beepActive = false;
  }
  if (desired != currentTone) {
    ledcWriteTone(PIN_BUZZER, desired);      // 0 stops the tone
    currentTone = desired;
  }
}

void handleEvents() {
  if (g_cmdSilence) {
    g_cmdSilence = false;
    if (state == ST_ALARM) {
      silenced = true;
      Serial.println("[CMD] alarm silenced");
    }
  }
  if (g_pirEvent) {
    g_pirEvent = false;
    beepActive = true;
    beepStart = millis();
    pirBlueUntil = millis() + 3000;
    Serial.println("[PIR] event from door board");
  }
}

void publishBlynk() {
  Blynk.virtualWrite(V0, temperatureC);
  Blynk.virtualWrite(V1, humidityPct);
  Blynk.virtualWrite(V2, coRaw);
  Blynk.virtualWrite(V3, flameRaw);
  Blynk.virtualWrite(V4, distanceCm);
  Blynk.virtualWrite(V5, stateName(state));
  Blynk.virtualWrite(V6, alarmText);
  Blynk.virtualWrite(V7, approachNear ? 1 : 0);
  Blynk.virtualWrite(V8, silenced ? 1 : 0);
}

BLYNK_WRITE(V9) {
  if (param.asInt()) sendTextPacket(PKT_CMD_SILENCE, "SILENCE");
}

// ============================== PUBLISHING ==============================
void publishChanges() {
  bool force = g_fullPublish;
  if (force) g_fullPublish = false;

  if (force || !havePub || state != pubState) {
    pub(T_STATE, stateName(state), true);
    pubState = state;
  }
  if (force || !havePub || strcmp(alarmText, pubAlarm)) {
    pub(T_ALARM, alarmText, true);
    strlcpy(pubAlarm, alarmText, sizeof(pubAlarm));
  }
  if (force || !havePub || silenced != pubSilenced) {
    pub(T_SILENCED, silenced ? "1" : "0", true);
    pubSilenced = silenced;
  }
  if (force || !havePub || approachNear != pubApproach) {
    pub(T_APPROACH, approachNear ? "1" : "0", true);
    pubApproach = approachNear;
  }
  havePub = true;
}

void publishReadings() {
  EspNowPacket p{};
  p.type = PKT_ENV_DATA;
  p.temperature = temperatureC;
  p.humidity = humidityPct;
  p.co = coRaw;
  p.flame = flameRaw;
  p.distance = distanceCm;
  sendPacket(p);
}

// ============================== SETUP / LOOP ==============================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n[BOOT] Environment board");

  espNowQ = xQueueCreate(24, sizeof(EspNowPacket));
  setupEspNow();
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(5000);

  dht.begin();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  analogSetPinAttenuation(PIN_CO, ADC_11db);
  analogSetPinAttenuation(PIN_FLAME, ADC_11db);

  ledcAttach(PIN_LED_R, 5000, 8);
  ledcAttach(PIN_LED_G, 5000, 8);
  ledcAttach(PIN_LED_B, 5000, 8);
  ledcAttach(PIN_BUZZER, 2000, 8);
  ledcWriteTone(PIN_BUZZER, 0);
  setRgb(0, 255, 0);

}

void loop() {
  static uint32_t tDht = 0, tAnalog = 0, tDist = 0, tPub = 0, tDebug = 0;
  uint32_t now = millis();
  Blynk.run();
  processEspNowPackets();
  static uint32_t tBlynk = 0;
  if (now - tBlynk >= 5000) {
    tBlynk = now;
    publishBlynk();
  }

  if (now - tDht >= 2000) {                 // DHT11 needs >= 1 s between reads
    tDht = now;
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (!isnan(h)) humidityPct = h;
    if (!isnan(t)) temperatureC = t;
  }

  if (now - tAnalog >= 200) {
    tAnalog = now;
    coRaw = readAdcAvg(PIN_CO);
    flameRaw = readAdcAvg(PIN_FLAME);
    evaluateAlarms();
  }

  if (now - tDist >= 200) {
    tDist = now;
    distanceCm = readDistanceCm();
    updateApproach();
  }

  handleEvents();
  updateRgb();
  updateBuzzer();
  publishChanges();

  if (now - tPub >= PUBLISH_INTERVAL_MS) {
    tPub = now;
    publishReadings();
  }

  if (now - tDebug >= 1000) {               // use these numbers to calibrate the thresholds
    tDebug = now;
    Serial.printf("T=%.1fC H=%.1f%% CO=%d flame=%d dist=%.0fcm state=%s alarm=%s%s\n",
                  temperatureC, humidityPct, coRaw, flameRaw, distanceCm,
                  stateName(state), alarmText, silenced ? " (silenced)" : "");
  }

  delay(5);
}
