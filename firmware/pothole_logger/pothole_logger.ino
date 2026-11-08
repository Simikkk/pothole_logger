/*
  Harta gropilor – bară de senzori pentru autobuz (ESP32)
  --------------------------------------------------------
  Hardware (într-o singură carcasă, montată sub bara din spate, ~40–50 cm de asfalt):
   - 5 × LiDAR TFmini-S pe I2C, în linie, la 20 cm unul de altul, orientați în jos;
   - accelerometru MPU6050 (aceeași magistrală I2C);
   - receptor GNSS generic NMEA pe UART (antenă externă pe acoperiș);
   - card microSD, LED de stare, buton de service;
   - alimentare din 24 V-ul autobuzului prin convertor DC-DC auto (5 V, min. 2 A).

  Cum măsoară o groapă:
   1. Fiecare senzor măsoară distanța până la asfalt de 200 de ori pe secundă.
   2. Fiecare senzor își învață încet nivelul drumului (baseline).
   3. Caroseria se leagănă pe suspensii și mută TOȚI senzorii deodată. Mediana celor 5
      abateri e exact această mișcare comună, așa că o scădem. O groapă e sub 1–2 senzori,
      deci nu strică mediana.
   4. Ce rămâne peste 2 cm = adâncitură. Cât timp durează (× viteza) = lungimea,
      câți senzori au prins-o (× 20 cm) = lățimea minimă.
   5. Poziția: ultimul fix GPS, dus înainte cu viteza × timpul scurs și înapoi cu distanța
      dintre antenă și bară (un autobuz are 12 m).

  Accelerometrul simte gropile pe care LiDAR-ul nu le poate măsura (pline cu apă sau în
  afara barei, dar sub roată). Un șoc e salvat doar dacă LiDAR-ul NU a măsurat în aceeași
  secundă o groapă, ca să nu fie numărată de două ori.

  Datele se trimit automat prin WiFi-ul router-ului din autobuz (pe nucleul 0, ca detecția
  de pe nucleul 1 să nu se oprească). Fără semnal, rămân pe card și pleacă la reconectare.

  Biblioteci (Arduino IDE -> Library Manager): TinyGPSPlus (Mikal Hart).
  Senzorii TFmini-S se configurează o singură dată, fiecare, cu tfmini_config.ino.
*/

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <TinyGPSPlus.h>
#include "config.h"

static const char* PENDING_FILE = "/pending.csv";
static const char* OUTBOX_FILE  = "/outbox.csv";

// ---------------------------------------------------------------- stare globală

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

SemaphoreHandle_t sdMutex;          // cardul e folosit din ambele nuclee
bool sdOk = false;

volatile bool uploadNow = true;
volatile uint32_t eventsLogged = 0, eventsSent = 0;

// Ultimul fix GPS și momentul în care a sosit
struct Fix { bool ok; double lat, lon; float speedMps, courseDeg; uint32_t unixTs, ms; };
Fix fix = {false, 0, 0, 0, 0, 0, 0};

// LiDAR
const uint8_t TF_ADDR[N_LIDAR] = LIDAR_ADDRESSES;
static const uint8_t TF_CMD_FRAME_MM[5] = {0x5A, 0x05, 0x00, 0x06, 0x65};  // „dă-mi o măsurătoare în mm”
float base[N_LIDAR];                // nivelul drumului învățat de fiecare senzor (mm)
bool  baseInit[N_LIDAR] = {false};
volatile uint32_t samplesTotal[N_LIDAR] = {0}, samplesValid[N_LIDAR] = {0};   // pentru „senzor murdar”
bool  lidarRequested = false;
uint32_t lastLidarMs = 0, lidarRequestMs = 0;

struct Pothole { bool active; uint32_t t0, lastHit; float top[3]; uint8_t mask; };
Pothole pot = {false, 0, 0, {0, 0, 0}, 0};
uint32_t lastLidarEventMs = 0;
uint32_t lastFlashMs = 0;          // LED aprins scurt la fiecare detecție

// Accelerometru
static const uint8_t MPU_ADDR = 0x68;
static const float   LSB_PER_G = 8192.0f;   // ±4g
float gx = 0, gy = 0, gz = 1.0f;
bool  gravityReady = false;
uint32_t warmupSamples = 0;
bool  inShock = false;
uint32_t shockStartMs = 0, lastShockMs = 0, lastImuMs = 0;
float shockPeak = 0;
struct PendingShock { bool active; uint32_t ms; float peak; Fix at; };
PendingShock pending = {false, 0, 0, {false, 0, 0, 0, 0, 0, 0}};

bool btnWasDown = false;
uint32_t btnDownMs = 0;

// ---------------------------------------------------------------- timp și poziție

int32_t daysFromCivil(int y, unsigned m, unsigned d) {      // algoritmul lui H. Hinnant
  y -= m <= 2;
  const int32_t era = (y >= 0 ? y : y - 399) / 400;
  const unsigned yoe = (unsigned)(y - era * 400);
  const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return era * 146097 + (int32_t)doe - 719468;
}

uint32_t gpsUnixTime() {
  if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < 2020) return 0;
  int32_t days = daysFromCivil(gps.date.year(), gps.date.month(), gps.date.day());
  return (uint32_t)days * 86400UL + gps.time.hour() * 3600UL + gps.time.minute() * 60UL + gps.time.second();
}

void readGps() {
  while (gpsSerial.available()) gps.encode(gpsSerial.read());
  if (gps.location.isUpdated() && gps.location.isValid()) {
    fix.lat = gps.location.lat();
    fix.lon = gps.location.lng();
    fix.speedMps = gps.speed.isValid() ? gps.speed.mps() : 0;
    if (gps.course.isValid() && fix.speedMps > 1.5f) fix.courseDeg = gps.course.deg();  // la oprire cursul e zgomot
    fix.unixTs = gpsUnixTime();
    fix.ms = millis();
    fix.ok = fix.unixTs > 0;
  }
}

bool fixFresh(const Fix &f) { return f.ok && millis() - f.ms < 3000; }
bool moving() { return fixFresh(fix) && fix.speedMps * 3.6f >= MIN_SPEED_KMH; }

// Poziția și ora unui eveniment petrecut la `ms`, pentru un punct aflat cu `backM` metri
// în spatele antenei: ducem fixul înainte cu viteza × timpul scurs, apoi înapoi cu backM.
void positionAt(const Fix &f, uint32_t ms, float backM, double &lat, double &lon, uint32_t &unixTs) {
  float dt = (int32_t)(ms - f.ms) / 1000.0f;
  float dist = f.speedMps * dt - backM;
  float c = radians(f.courseDeg);
  lat = f.lat + dist * cosf(c) / 111320.0;
  lon = f.lon + dist * sinf(c) / (111320.0 * cos(radians(f.lat)));
  unixTs = f.unixTs + (int32_t)lroundf(dt);
}

// ---------------------------------------------------------------- card SD

void logLine(const char *line) {
  bool written = false;
  if (sdOk) {
    xSemaphoreTake(sdMutex, portMAX_DELAY);
    File f = SD.open(PENDING_FILE, FILE_APPEND);
    if (f) { f.println(line); f.close(); written = true; }
    xSemaphoreGive(sdMutex);
  }
  if (!written) Serial.println("[eroare] nu pot scrie pe SD");
  eventsLogged++;
  Serial.println(line);
  digitalWrite(PIN_LED, HIGH);      // aprindere scurtă la fiecare detecție
  lastFlashMs = millis();
}

// ---------------------------------------------------------------- LiDAR

void tfRequestAll() {
  for (int i = 0; i < N_LIDAR; i++) {
    Wire.beginTransmission(TF_ADDR[i]);
    Wire.write(TF_CMD_FRAME_MM, sizeof(TF_CMD_FRAME_MM));
    Wire.endTransmission();
  }
}

// Cadru TFmini-S: 0x59 0x59 distL distH forțăL forțăH tempL tempH suma
bool tfRead(int i, uint16_t &mm, uint16_t &strength) {
  if (Wire.requestFrom(TF_ADDR[i], (uint8_t)9) != 9) {
    while (Wire.available()) Wire.read();
    return false;
  }
  uint8_t b[9];
  for (int j = 0; j < 9; j++) b[j] = Wire.read();
  uint8_t sum = 0;
  for (int j = 0; j < 8; j++) sum += b[j];
  if (b[0] != 0x59 || b[1] != 0x59 || sum != b[8]) return false;
  mm = b[2] | (b[3] << 8);
  strength = b[4] | (b[5] << 8);
  return true;
}

float median(float *v, int n) {             // sortare prin inserție, n ≤ 5
  for (int i = 1; i < n; i++) {
    float x = v[i]; int j = i - 1;
    while (j >= 0 && v[j] > x) { v[j + 1] = v[j]; j--; }
    v[j + 1] = x;
  }
  return (n % 2) ? v[n / 2] : (v[n / 2 - 1] + v[n / 2]) / 2;
}

void pushTop(float d) {                    // păstrăm cele mai mari 3 adâncimi (max robust)
  if (d > pot.top[0]) { pot.top[2] = pot.top[1]; pot.top[1] = pot.top[0]; pot.top[0] = d; }
  else if (d > pot.top[1]) { pot.top[2] = pot.top[1]; pot.top[1] = d; }
  else if (d > pot.top[2]) { pot.top[2] = d; }
}

void finishPothole() {
  pot.active = false;
  float speed = fix.speedMps;
  float lengthCm = (pot.lastHit - pot.t0) / 1000.0f * speed * 100 + speed * 100 / LIDAR_HZ;
  if (lengthCm < POTHOLE_MIN_LEN_CM || lengthCm > POTHOLE_MAX_LEN_CM || !fixFresh(fix)) return;

  int lo = 0, hi = N_LIDAR - 1;
  while (lo < N_LIDAR && !(pot.mask & (1 << lo))) lo++;
  while (hi >= 0 && !(pot.mask & (1 << hi))) hi--;
  float widthCm = (hi - lo + 1) * LIDAR_SPACING_CM;
  float depthCm = (pot.top[1] > 0 ? pot.top[1] : pot.top[0]) / 10.0f;   // a doua cea mai mare: ignoră un vârf izolat

  double lat, lon; uint32_t unixTs;
  positionAt(fix, (pot.t0 + pot.lastHit) / 2, ANTENNA_TO_BAR_M, lat, lon, unixTs);
  char line[112];
  snprintf(line, sizeof(line), "L,%lu,%.6f,%.6f,%.1f,%.1f,%.0f,%.0f",
           (unsigned long)unixTs, lat, lon, speed * 3.6f, depthCm, lengthCm, widthCm);
  logLine(line);
  lastLidarEventMs = millis();

  // roata (în fața barei) a lovit probabil aceeași groapă puțin mai devreme: nu o dublăm
  if (pending.active && pending.ms + SHOCK_HOLD_MS >= pot.t0) pending.active = false;
}

void processLidar(uint32_t sampleMs) {
  float rel[N_LIDAR], tmp[N_LIDAR];
  bool ok[N_LIDAR];
  int nValid = 0;
  bool mov = moving();

  for (int i = 0; i < N_LIDAR; i++) {
    uint16_t mm = 0, st = 0;
    ok[i] = tfRead(i, mm, st) && st >= LIDAR_MIN_STRENGTH && st != 65535 &&
            mm >= LIDAR_MIN_RANGE_MM && mm <= LIDAR_MAX_RANGE_MM;
    if (mov) { samplesTotal[i]++; if (ok[i]) samplesValid[i]++; }
    if (!ok[i]) continue;
    if (!baseInit[i]) { base[i] = mm; baseInit[i] = true; }
    rel[i] = mm - base[i];
    tmp[nValid++] = rel[i];
  }

  if (nValid < 3 || !mov) {                 // prea puține citiri bune sau autobuz oprit
    if (pot.active) finishPothole();
    return;
  }

  float common = median(tmp, nValid);       // mișcarea caroseriei, comună tuturor senzorilor
  uint8_t hitMask = 0;
  float sampleMax = 0;
  for (int i = 0; i < N_LIDAR; i++) {
    if (!ok[i]) continue;
    float depth = rel[i] - common;          // mm; pozitiv = asfaltul e mai departe = adâncitură
    if (depth >= DEPTH_MIN_MM) {
      hitMask |= 1 << i;
      if (depth > sampleMax) sampleMax = depth;
    } else if (!pot.active) {
      base[i] += BASELINE_ALPHA * depth;    // învățăm nivelul drumului doar pe asfalt normal
    }
  }

  if (hitMask) {
    if (!pot.active) {
      pot.active = true; pot.t0 = sampleMs; pot.mask = 0;
      pot.top[0] = pot.top[1] = pot.top[2] = 0;
    }
    pot.lastHit = sampleMs;
    pot.mask |= hitMask;
    pushTop(sampleMax);
  } else if (pot.active && (sampleMs - pot.lastHit) / 1000.0f * fix.speedMps * 100 >= POTHOLE_GAP_CM) {
    finishPothole();
  }
}

// ---------------------------------------------------------------- accelerometru

void mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpuInit() {
  mpuWrite(0x6B, 0x00);   // trezire din sleep
  mpuWrite(0x1C, 0x08);   // ±4g
  mpuWrite(0x1A, 0x03);   // filtru trece-jos intern ~44 Hz
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);       // WHO_AM_I
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)1);
  return Wire.available() && Wire.read() == 0x68;
}

bool mpuRead(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  ax = (int16_t)((b[0] << 8) | b[1]) / LSB_PER_G;
  ay = (int16_t)((b[2] << 8) | b[3]) / LSB_PER_G;
  az = (int16_t)((b[4] << 8) | b[5]) / LSB_PER_G;
  return true;
}

void processImu(float ax, float ay, float az) {
  if (!gravityReady) {
    float a = 1.0f / (warmupSamples + 1);
    gx += (ax - gx) * a; gy += (ay - gy) * a; gz += (az - gz) * a;
    if (++warmupSamples >= 200) gravityReady = true;
    return;
  }
  float dx = ax - gx, dy = ay - gy, dz = az - gz;
  float dev = sqrtf(dx * dx + dy * dy + dz * dz);
  if (!inShock && dev < SHOCK_THRESHOLD_G) {
    gx += (ax - gx) * GRAVITY_EMA_ALPHA;
    gy += (ay - gy) * GRAVITY_EMA_ALPHA;
    gz += (az - gz) * GRAVITY_EMA_ALPHA;
  }

  uint32_t now = millis();
  if (!inShock) {
    if (dev >= SHOCK_THRESHOLD_G && moving() && now - lastShockMs > REFRACTORY_MS) {
      inShock = true; shockStartMs = now; shockPeak = dev;
    }
  } else {
    if (dev > shockPeak) shockPeak = dev;
    if (now - shockStartMs >= EVENT_WINDOW_MS) {
      inShock = false;
      lastShockMs = now;
      // dacă LiDAR-ul tocmai a măsurat o groapă (bară montată în față), e aceeași
      if (now - lastLidarEventMs < SHOCK_HOLD_MS) return;
      pending = {true, shockStartMs, shockPeak, fix};   // așteptăm să vedem dacă vine LiDAR-ul
    }
  }
}

void flushPendingShock() {
  if (!pending.active || millis() - pending.ms < SHOCK_HOLD_MS) return;
  pending.active = false;
  if (!pending.at.ok) return;
  double lat, lon; uint32_t unixTs;
  positionAt(pending.at, pending.ms, ANTENNA_TO_REAR_AXLE_M, lat, lon, unixTs);
  char line[96];
  snprintf(line, sizeof(line), "S,%lu,%.6f,%.6f,%.1f,%.3f",
           (unsigned long)unixTs, lat, lon, pending.at.speedMps * 3.6f, pending.peak);
  logLine(line);
}

// ---------------------------------------------------------------- trimitere (nucleul 0)

// Linie de pe card -> obiect JSON pentru server
bool csvToJson(const String &line, String &out) {
  unsigned long ts; double lat, lon; float v, a, b, c;
  char buf[160];
  if (line.startsWith("L,")) {
    if (sscanf(line.c_str() + 2, "%lu,%lf,%lf,%f,%f,%f,%f", &ts, &lat, &lon, &v, &a, &b, &c) != 7) return false;
    snprintf(buf, sizeof(buf), "{\"t\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"v\":%.1f,\"k\":\"lidar\",\"d\":%.1f,\"l\":%.0f,\"w\":%.0f}",
             ts, lat, lon, v, a, b, c);
  } else if (line.startsWith("S,")) {
    if (sscanf(line.c_str() + 2, "%lu,%lf,%lf,%f,%f", &ts, &lat, &lon, &v, &a) != 5) return false;
    snprintf(buf, sizeof(buf), "{\"t\":%lu,\"lat\":%.6f,\"lon\":%.6f,\"v\":%.1f,\"k\":\"soc\",\"g\":%.3f}",
             ts, lat, lon, v, a);
  } else {
    return false;
  }
  out = buf;
  return true;
}

bool postJson(const char *path, const String &body) {
  HTTPClient http;
  WiFiClient plain;
  WiFiClientSecure secure;
  String url = String(SERVER_BASE) + path;
  bool started;
  if (url.startsWith("https://")) {
    if (strlen(ROOT_CA) > 0) secure.setCACert(ROOT_CA);
    else if (ALLOW_INSECURE_TLS) secure.setInsecure();
    else { Serial.println("[upload] https fara ROOT_CA in config.h - refuz conexiunea"); return false; }
    started = http.begin(secure, url);
  } else {
    started = http.begin(plain, url);
  }
  if (!started) return false;
  http.setConnectTimeout(5000);
  http.setTimeout(8000);
  http.addHeader("Content-Type", "application/json");
  if (strlen(API_KEY) > 0) http.addHeader("X-API-Key", API_KEY);
  int code = http.POST(body);
  http.end();
  if (code != 200) Serial.printf("[upload] %s: HTTP %d\n", path, code);
  return code == 200;
}

bool prepareOutbox() {
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  if (!SD.exists(OUTBOX_FILE) && SD.exists(PENDING_FILE)) SD.rename(PENDING_FILE, OUTBOX_FILE);
  bool has = SD.exists(OUTBOX_FILE);
  xSemaphoreGive(sdMutex);
  return has;
}

size_t readBatch(size_t pos, String &batch, int &count, bool &eof) {
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  File f = SD.open(OUTBOX_FILE, FILE_READ);
  if (!f) { xSemaphoreGive(sdMutex); eof = true; return pos; }
  f.seek(pos);
  String item;
  while (f.available() && count < UPLOAD_BATCH) {
    String line = f.readStringUntil('\n');
    line.trim();
    if (line.length() == 0 || !csvToJson(line, item)) continue;
    if (count > 0) batch += ",";
    batch += item;
    count++;
  }
  size_t next = f.position();
  eof = !f.available();
  f.close();
  xSemaphoreGive(sdMutex);        // eliberăm cardul înainte de rețea
  return next;
}

bool uploadOutbox() {
  if (!sdOk || WiFi.status() != WL_CONNECTED) return false;
  if (!prepareOutbox()) return true;
  size_t pos = 0;
  int total = 0;
  bool eof = false;
  while (!eof) {
    String batch;
    int count = 0;
    pos = readBatch(pos, batch, count, eof);
    if (count > 0) {
      String body = String("{\"device_id\":\"") + DEVICE_ID + "\",\"events\":[" + batch + "]}";
      if (!postJson("/api/events", body)) return false;   // rămâne în outbox; serverul ignoră duplicatele
      total += count;
    }
  }
  xSemaphoreTake(sdMutex, portMAX_DELAY);
  SD.remove(OUTBOX_FILE);
  xSemaphoreGive(sdMutex);
  eventsSent += total;
  Serial.printf("[upload] %d detectii trimise\n", total);
  return true;
}

// Semn de viață: ce procent din citirile fiecărui senzor au fost bune de la ultimul heartbeat
void sendHeartbeat() {
  static uint32_t prevTotal[N_LIDAR] = {0}, prevValid[N_LIDAR] = {0};
  String body = String("{\"device_id\":\"") + DEVICE_ID + "\",\"fw\":\"" + FW_VERSION + "\",\"gps\":" +
                (fixFresh(fix) ? "true" : "false") + ",\"pending\":" + String(eventsLogged - eventsSent) +
                ",\"sensors\":[";
  for (int i = 0; i < N_LIDAR; i++) {
    uint32_t t = samplesTotal[i], v = samplesValid[i];
    uint32_t dt = t - prevTotal[i], dv = v - prevValid[i];
    prevTotal[i] = t; prevValid[i] = v;
    if (i > 0) body += ",";
    // sub 200 de citiri în mers (autobuz parcat) nu putem judeca; serverul presupune „ok”
    body += dt >= 200 ? String("{\"valid\":") + String((float)dv / dt, 2) + "}" : String("{}");
  }
  body += "]}";
  postJson("/api/heartbeat", body);
}

void uploadTask(void *) {
  uint32_t lastTry = 0, lastBeat = 0, lastWifiAttempt = millis();
  bool firstBeat = true;
  for (;;) {
    uint32_t now = millis();
    if (WiFi.status() != WL_CONNECTED) {
      if (now - lastWifiAttempt >= WIFI_RETRY_MS) {
        lastWifiAttempt = now;
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
      }
    } else {
      if (uploadNow || now - lastTry >= UPLOAD_INTERVAL_MS) {
        uploadNow = false;
        lastTry = now;
        uploadOutbox();
      }
      if (firstBeat || now - lastBeat >= HEARTBEAT_MS) {
        firstBeat = false;
        lastBeat = now;
        sendHeartbeat();
      }
    }
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// ---------------------------------------------------------------- buton și LED

void handleButton() {
  bool down = digitalRead(PIN_BUTTON) == LOW;
  if (down && !btnWasDown) btnDownMs = millis();
  if (!down && btnWasDown && millis() - btnDownMs >= 2000) uploadNow = true;   // service: trimite acum
  btnWasDown = down;
}

//  clipire 1 Hz = caută GPS  |  scurt la 2 s = fără WiFi  |  stins = ok, aprindere scurtă la fiecare groapă
void updateStatusLed(uint32_t now) {
  if (!fixFresh(fix))                      digitalWrite(PIN_LED, (now / 500) % 2);
  else if (WiFi.status() != WL_CONNECTED)  digitalWrite(PIN_LED, (now % 2000) < 80);
  else if (now - lastFlashMs > 150)        digitalWrite(PIN_LED, LOW);
}

// ---------------------------------------------------------------- setup / loop

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  sdMutex = xSemaphoreCreateMutex();

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, PIN_GPS_TX);
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);

  if (!mpuInit()) Serial.println("[eroare] MPU6050 nu raspunde - continui doar cu LiDAR");
  for (int i = 0; i < N_LIDAR; i++) {
    uint16_t mm, st;
    tfRequestAll();
    delay(5);
    bool ok = tfRead(i, mm, st);
    Serial.printf("[lidar %d] adresa 0x%02X: %s\n", i + 1, TF_ADDR[i], ok ? "ok" : "NU RASPUNDE");
  }

  sdOk = SD.begin(PIN_SD_CS);
  if (!sdOk) Serial.println("[eroare] card SD absent - datele NU vor fi salvate");

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  xTaskCreatePinnedToCore(uploadTask, "upload", 12288, nullptr, 1, nullptr, 0);

  Serial.printf("[ok] %s pornit, fw %s\n", DEVICE_ID, FW_VERSION);
}

void loop() {
  readGps();
  uint32_t now = millis();

  // LiDAR: citim răspunsurile cererii anterioare, apoi cerem următoarea măsurătoare.
  // Așa senzorii au o perioadă întreagă să răspundă și bucla nu așteaptă niciodată.
  if (now - lastLidarMs >= 1000 / LIDAR_HZ) {
    lastLidarMs = now;
    if (lidarRequested) processLidar(lidarRequestMs);
    tfRequestAll();
    lidarRequestMs = now;
    lidarRequested = true;
  }

  if (now - lastImuMs >= SAMPLE_PERIOD_MS) {
    lastImuMs = now;
    float ax, ay, az;
    if (mpuRead(ax, ay, az)) processImu(ax, ay, az);
  }

  flushPendingShock();
  handleButton();
  updateStatusLed(now);
}
