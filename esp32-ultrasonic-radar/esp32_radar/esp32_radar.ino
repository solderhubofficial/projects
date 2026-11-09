// ================================================================
//  ESP32 Ultrasonic Radar: smooth sweep, web dashboard, 1.3" OLED
//
//  SolderHub | https://solderhub.com
//  A community for electronics enthusiasts and makers.
//  Step-by-step guide:
//  https://solderhub.com/projects/esp32-servo-ultrasonic-radar-web-dashboard
//  Source code: https://github.com/solderhubofficial/projects
//
//  Board:    ESP32 Dev Module (WROOM-32)
//  Sensor:   HC-SR04   Servo: SG90   Display: SH1106 1.3" I2C (optional)
//  Libraries: ESP32Servo, Adafruit SH110X, Adafruit GFX
//
//  Wiring
//    HC-SR04 VCC -> 5V (VIN)      GND -> GND
//    HC-SR04 TRIG -> GPIO5
//    HC-SR04 ECHO -> 1k -> GPIO18, and 2k from GPIO18 to GND
//    SG90 orange -> GPIO15        red -> external 5V
//    SG90 brown  -> external GND, tied to ESP32 GND
//    OLED VCC -> 3V3  GND -> GND  SDA -> GPIO21  SCL -> GPIO22
//    Active buzzer + -> GPIO4     - -> GND
// ================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <esp_system.h>

#define USE_OLED 1  // Set to 0 to build without the display

#if USE_OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#endif

// ---------- Edit these ----------
const char* WIFI_SSID = "YOUR_WIFI_SSID";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";

// If the router can't be reached, the ESP32 creates this network instead.
const char* AP_SSID = "ESP32-Radar";
const char* AP_PASS = "radar1234";

const uint32_t SWEEP_ONE_WAY_MS = 3000;  // Higher = slower and smoother
const int SWEEP_MIN = 15;                // degrees
const int SWEEP_MAX = 165;               // degrees
const float MAX_CM = 200;                // Radar's outer ring
const bool FLIP_DISPLAY = false;         // true if left and right look swapped
const float BUZZER_ALERT_CM = 50;        // Beeps inside this range, 0 = silent
// --------------------------------

const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int SERVO_PIN = 15;
const int BUZZER_PIN = 4;
const int OLED_SDA = 21;
const int OLED_SCL = 22;
const uint8_t OLED_ADDR = 0x3C;

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2400;
const uint32_t SERVO_FRAME_MS = 20;  // The servo only accepts 50 updates a second
const uint32_t PING_MS = 70;         // Gives each echo time to die away
const unsigned long ECHO_TIMEOUT_US = 30000;
const float NO_ECHO = 9999;

const uint32_t SWEEP_PERIOD_MS = 2 * SWEEP_ONE_WAY_MS;
const float SWEEP_MID = (SWEEP_MIN + SWEEP_MAX) / 2.0f;
const float SWEEP_AMP = (SWEEP_MAX - SWEEP_MIN) / 2.0f;

struct Reading {
  float angle;
  float cm;
};

const int RING_SIZE = 64;
Reading ring[RING_SIZE];
uint32_t ringSeq = 0;
float latestCm = -1;
portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

uint32_t sweepStart = 0;

Servo radarServo;
WebServer server(80);

uint32_t sweepMs() {
  return millis() - sweepStart;
}

// Position is a smooth cosine of time: no stop-and-go, and gentle reversals.
float angleAt(uint32_t t) {
  float phase = float(t % SWEEP_PERIOD_MS) / SWEEP_PERIOD_MS;
  return SWEEP_MID - SWEEP_AMP * cos(TWO_PI * phase);
}

int angleToMicros(float angle) {
  return SERVO_MIN_US + angle / 180.0f * (SERVO_MAX_US - SERVO_MIN_US);
}

// Highest priority, fixed rate, so nothing else can make the servo stutter.
void motionTask(void*) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(SERVO_FRAME_MS));
    radarServo.writeMicroseconds(angleToMicros(angleAt(sweepMs())));
  }
}

float pingCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (us == 0) return NO_ECHO;

  float cm = us * 0.0343f / 2.0f;
  return (cm < 2 || cm > MAX_CM) ? NO_ECHO : cm;
}

float median3(float a, float b, float c) {
  return max(min(a, b), min(max(a, b), c));
}

void sensorTask(void*) {
  float cm[3] = {NO_ECHO, NO_ECHO, NO_ECHO};
  float angle[3] = {SWEEP_MIN, SWEEP_MIN, SWEEP_MIN};
  TickType_t wake = xTaskGetTickCount();

  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PING_MS));

    cm[0] = cm[1];
    cm[1] = cm[2];
    angle[0] = angle[1];
    angle[1] = angle[2];
    angle[2] = angleAt(sweepMs());
    cm[2] = pingCm();

    // A median of 3 removes one-off ghost echoes. It describes the middle ping.
    float filtered = median3(cm[0], cm[1], cm[2]);
    bool valid = filtered < NO_ECHO;

    portENTER_CRITICAL(&ringMux);
    latestCm = valid ? filtered : -1;
    if (valid) {
      ring[ringSeq % RING_SIZE] = {angle[1], filtered};
      ringSeq++;
    }
    portEXIT_CRITICAL(&ringMux);
  }
}

// Copies readings newer than `since`, then moves `since` forward.
int copyReadings(uint32_t& since, Reading* out, float* latest) {
  int count = 0;
  portENTER_CRITICAL(&ringMux);
  if (since > ringSeq) since = 0;
  if (ringSeq - since > RING_SIZE) since = ringSeq - RING_SIZE;
  while (since < ringSeq) {
    out[count++] = ring[since % RING_SIZE];
    since++;
  }
  if (latest) *latest = latestCm;
  portEXIT_CRITICAL(&ringMux);
  return count;
}

const char INDEX_HTML[] PROGMEM = R"HTMLPAGE(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Radar</title>
<style>
  body{background:#0b0f0b;color:#39ff14;font-family:monospace;text-align:center;margin:0;padding:16px;}
  h1{font-size:20px;letter-spacing:2px;margin:4px 0;}
  #readout{font-size:16px;margin:10px 0;color:#7dffb3;}
  #link{font-size:12px;margin:8px 0;color:#7dffb3;}
  canvas{background:#040704;border:2px solid #1a3d1a;border-radius:8px;max-width:100%;height:auto;}
</style>
</head>
<body>
<h1>ESP32 RADAR</h1>
<div id="readout">ANGLE: -- &deg;&nbsp;&nbsp; DIST: -- cm</div>
<canvas id="radar" width="600" height="320"></canvas>
<div id="link">CONNECTING</div>
<script>
const canvas = document.getElementById('radar');
const ctx = canvas.getContext('2d');
const cx = 300, cy = 310, R = 290, BLIP_MS = 5000, TRAIL = 32, TRAIL_STEP = 25;
let cfg = null, since = 0, clockOffset = null, blips = [], latest = -1;

function angleAt(t){
  const p = cfg.period, u = ((t % p) + p) % p;
  return cfg.mid - cfg.amp * Math.cos(2 * Math.PI * u / p);
}

function point(deg, r){
  const a = deg * Math.PI / 180, s = cfg.flip ? -1 : 1;
  return [cx - s * r * Math.cos(a), cy - r * Math.sin(a)];
}

function line(deg, color, width){
  const p = point(deg, R);
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(cx, cy);
  ctx.lineTo(p[0], p[1]);
  ctx.stroke();
}

function drawGrid(){
  ctx.strokeStyle = '#1a3d1a';
  ctx.lineWidth = 1;
  ctx.fillStyle = '#3f8f3f';
  ctx.font = '11px monospace';
  for(let i = 1; i <= 4; i++){
    const r = R * i / 4;
    ctx.beginPath();
    ctx.arc(cx, cy, r, Math.PI, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(Math.round(cfg.maxCm * i / 4) + 'cm', cx + 4, cy - r + 12);
  }
  for(let a = 0; a <= 180; a += 30) line(a, '#1a3d1a', 1);
}

function drawBlips(now){
  blips = blips.filter(b => now - b.t < BLIP_MS);
  blips.forEach(b => {
    const p = point(b.a, Math.min(b.d / cfg.maxCm, 1) * R);
    ctx.fillStyle = `rgba(255,60,60,${1 - (now - b.t) / BLIP_MS})`;
    ctx.beginPath();
    ctx.arc(p[0], p[1], 4, 0, 2 * Math.PI);
    ctx.fill();
  });
}

function drawSweep(t){
  for(let k = TRAIL; k > 0; k--){
    line(angleAt(t - k * TRAIL_STEP), `rgba(57,255,20,${0.28 * (1 - k / TRAIL)})`, 3);
  }
  line(angleAt(t), 'rgba(57,255,20,0.95)', 3);
}

function render(){
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  if(cfg){
    const now = performance.now(), t = now + clockOffset, angle = angleAt(t);
    drawGrid();
    drawBlips(now);
    drawSweep(t);
    document.getElementById('readout').innerHTML =
      'ANGLE: ' + angle.toFixed(0) + '&deg;&nbsp;&nbsp; DIST: ' +
      (latest < 0 ? '--' : latest.toFixed(1)) + ' cm';
  }
  requestAnimationFrame(render);
}
render();

async function poll(){
  let wait = 100;
  try{
    const j = await (await fetch('/data?since=' + since)).json();
    const now = performance.now(), offset = j.t - now;
    cfg = j;
    since = j.seq;
    latest = j.d;
    clockOffset = (clockOffset === null || Math.abs(offset - clockOffset) > 2000)
      ? offset : clockOffset + (offset - clockOffset) * 0.1;
    j.r.forEach(r => blips.push({a: r[0], d: r[1], t: now}));
    document.getElementById('link').textContent = 'LIVE';
  }catch(e){
    document.getElementById('link').textContent = 'CONNECTION LOST, RETRYING';
    wait = 500;
  }
  setTimeout(poll, wait);
}
poll();
</script>
</body>
</html>
)HTMLPAGE";

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleData() {
  uint32_t since = strtoul(server.arg("since").c_str(), nullptr, 10);
  Reading readings[RING_SIZE];
  float latest = -1;
  int count = copyReadings(since, readings, &latest);

  String json;
  json.reserve(180 + count * 14);
  json += "{\"t\":";
  json += sweepMs();
  json += ",\"seq\":";
  json += since;
  json += ",\"period\":";
  json += SWEEP_PERIOD_MS;
  json += ",\"mid\":";
  json += String(SWEEP_MID, 1);
  json += ",\"amp\":";
  json += String(SWEEP_AMP, 1);
  json += ",\"maxCm\":";
  json += String(MAX_CM, 0);
  json += ",\"flip\":";
  json += FLIP_DISPLAY ? "true" : "false";
  json += ",\"d\":";
  json += String(latest, 1);
  json += ",\"r\":[";
  for (int i = 0; i < count; i++) {
    if (i) json += ',';
    json += '[';
    json += String(readings[i].angle, 1);
    json += ',';
    json += String(readings[i].cm, 1);
    json += ']';
  }
  json += "]}";
  server.send(200, "application/json", json);
}

// Web requests run on core 0, away from the servo and sensor tasks on core 1.
void webTask(void*) {
  for (;;) {
    server.handleClient();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// Beeps faster as the nearest object gets closer.
void buzzerTask(void*) {
  for (;;) {
    portENTER_CRITICAL(&ringMux);
    float cm = latestCm;
    portEXIT_CRITICAL(&ringMux);

    if (cm > 0 && cm <= BUZZER_ALERT_CM) {
      digitalWrite(BUZZER_PIN, HIGH);
      vTaskDelay(pdMS_TO_TICKS(40));
      digitalWrite(BUZZER_PIN, LOW);
      vTaskDelay(pdMS_TO_TICKS(60 + cm / BUZZER_ALERT_CM * 440));
    } else {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

#if USE_OLED
const int OLED_CX = 64, OLED_CY = 63, OLED_R = 60;
const uint32_t OLED_FRAME_MS = 100;
const uint32_t OLED_BLIP_MS = 4000;
const int OLED_BUCKETS = 46;  // One per 4 degrees

Adafruit_SH1106G display(128, 64, &Wire, -1);
bool oledReady = false;
uint32_t oledSince = 0;
uint32_t oledHoldUntil = 0;
float bucketCm[OLED_BUCKETS];
uint32_t bucketAt[OLED_BUCKETS];

void oledText(const char* a, const String& b, const String& c) {
  if (!oledReady) return;
  display.clearDisplay();
  display.setCursor(0, 8);
  display.println(a);
  display.setCursor(0, 28);
  display.println(b);
  display.setCursor(0, 44);
  display.println(c);
  display.display();
}

void oledPoint(float angle, float radius, int& x, int& y) {
  float a = radians(angle);
  int side = FLIP_DISPLAY ? -1 : 1;
  x = OLED_CX - side * radius * cos(a);
  y = OLED_CY - radius * sin(a);
}

void updateOled() {
  static uint32_t lastFrame = 0;
  uint32_t now = millis();
  if (!oledReady || int32_t(now - oledHoldUntil) < 0 || now - lastFrame < OLED_FRAME_MS) return;
  lastFrame = now;

  Reading fresh[RING_SIZE];
  float latest = -1;
  int count = copyReadings(oledSince, fresh, &latest);
  for (int i = 0; i < count; i++) {
    int bucket = constrain(int(fresh[i].angle / 4), 0, OLED_BUCKETS - 1);
    bucketCm[bucket] = fresh[i].cm;
    bucketAt[bucket] = now;
  }

  float angle = angleAt(sweepMs());
  display.clearDisplay();
  display.drawCircleHelper(OLED_CX, OLED_CY, OLED_R, 0x3, SH110X_WHITE);
  display.drawCircleHelper(OLED_CX, OLED_CY, OLED_R / 2, 0x3, SH110X_WHITE);

  int x, y;
  oledPoint(angle, OLED_R, x, y);
  display.drawLine(OLED_CX, OLED_CY, x, y, SH110X_WHITE);

  for (int i = 0; i < OLED_BUCKETS; i++) {
    if (bucketAt[i] == 0 || now - bucketAt[i] > OLED_BLIP_MS) continue;
    oledPoint(i * 4 + 2, bucketCm[i] / MAX_CM * OLED_R, x, y);
    display.fillCircle(x, y, 1, SH110X_WHITE);
  }

  display.setCursor(0, 0);
  if (latest < 0) {
    display.print("--");
  } else {
    display.print(int(latest));
    display.print("cm");
  }
  String deg = String(int(angle)) + "d";
  display.setCursor(128 - deg.length() * 6, 0);
  display.print(deg);
  display.display();
}

// The OLED runs on core 0 because the servo task keeps core 1 busy.
void oledTask(void*) {
  Wire.end();
  Wire.begin(OLED_SDA, OLED_SCL);  // Moves I2C interrupts to this core
  for (;;) {
    updateOled();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
#endif

void showStatus(const char* a, const String& b, const String& c, uint32_t holdMs) {
  Serial.printf("%s %s %s\n", a, b.c_str(), c.c_str());
#if USE_OLED
  oledText(a, b, c);
  oledHoldUntil = millis() + holdMs;
#endif
}

void startNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // Lower, steadier dashboard latency
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  showStatus("Connecting WiFi", WIFI_SSID, "", 0);

  uint32_t started = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - started < 15000) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    showStatus("Open in browser:", WiFi.localIP().toString(), "", 5000);
    return;
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  showStatus("Own WiFi mode", String(AP_SSID) + " / " + AP_PASS,
             WiFi.softAPIP().toString(), 8000);
}

void setup() {
  Serial.begin(115200);
  delay(200);

  if (esp_reset_reason() == ESP_RST_BROWNOUT) {
    Serial.println("Brownout reset: give the servo its own 5V supply.");
  }

  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

#if USE_OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  oledReady = display.begin(OLED_ADDR, true);
  if (oledReady) {
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    showStatus("Starting radar", "", "", 0);
  } else {
    Serial.println("OLED not found, continuing without it.");
  }
#endif

  // Keep this order: Wi-Fi startup froze when it ran after the tasks below.
  startNetwork();
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();
  xTaskCreatePinnedToCore(webTask, "web", 8192, nullptr, 1, nullptr, 0);

  ESP32PWM::allocateTimer(0);
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  radarServo.writeMicroseconds(angleToMicros(SWEEP_MIN));
  delay(700);  // Let the servo reach the start before sweeping

  sweepStart = millis();
  xTaskCreatePinnedToCore(motionTask, "motion", 4096, nullptr, 3, nullptr, 1);
  xTaskCreatePinnedToCore(sensorTask, "sensor", 4096, nullptr, 2, nullptr, 0);  // Core 0: no pings ever completed on core 1
  xTaskCreatePinnedToCore(buzzerTask, "buzzer", 2048, nullptr, 1, nullptr, 0);
#if USE_OLED
  if (oledReady) xTaskCreatePinnedToCore(oledTask, "oled", 8192, nullptr, 1, nullptr, 0);
#endif
}

void loop() {
  // All the work happens in the FreeRTOS tasks started in setup().
  vTaskDelay(pdMS_TO_TICKS(1000));
}
