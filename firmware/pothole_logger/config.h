// ===================== Configurare bară de senzori (autobuz) =====================
// Modifică valorile de mai jos înainte de a încărca codul pe ESP32.
#pragma once

#define FW_VERSION     "2.0"
#define DEVICE_ID      "bus-001"     // un cod per autobuz (de ex. numărul de parc)

// --- Rețea: router-ul de bord al autobuzului ---
// Rețeaua DEDICATĂ dispozitivelor (cere-o la operator), NU cea pentru pasageri.
#define WIFI_SSID      "RATBV-IoT"
#define WIFI_PASS      "ParolaWiFi"

// --- Server ---
// Test pe rețeaua de acasă: "http://192.168.1.100:5000"  (IP-ul laptopului, fără / la final)
// Pe autobuz, prin internet: obligatoriu https://, de ex. "https://gropi.exemplu.ro"
#define SERVER_BASE    "http://192.168.1.100:5000"
#define API_KEY        ""            // aceeași cheie ca POTHOLE_API_KEY pe server

// Certificatul rădăcină al serverului, pentru https://. Fără el, ESP32 refuză conexiunea.
// Îl obții din browser: lacătul din bara de adrese -> certificat -> rădăcina lanțului ->
// export PEM (Base64), apoi îl lipești aici.
static const char ROOT_CA[] = "";
#define ALLOW_INSECURE_TLS   0       // doar pentru teste; NU lăsa 1 pe autobuz

#define UPLOAD_INTERVAL_MS   60000   // trimite detecțiile noi o dată pe minut
#define HEARTBEAT_MS         300000  // semn de viață + starea senzorilor la 5 minute
#define UPLOAD_BATCH         50
#define WIFI_RETRY_MS        30000

// --- Bara LiDAR: senzori TFmini-S pe I2C (adresele se setează o dată cu tfmini_config.ino) ---
#define N_LIDAR              5
#define LIDAR_ADDRESSES      {0x10, 0x11, 0x12, 0x13, 0x14}   // de la stânga la dreapta
#define LIDAR_SPACING_CM     20      // distanța dintre senzori pe bară
#define LIDAR_HZ             200     // citiri pe secundă (la 50 km/h: o citire la ~7 cm)
#define LIDAR_MIN_STRENGTH   100     // sub asta citirea e nesigură (apă, murdărie, unghi)
#define LIDAR_MIN_RANGE_MM   150
#define LIDAR_MAX_RANGE_MM   1500

// Detecția gropilor
#define DEPTH_MIN_MM         20      // adâncitură minimă luată în seamă (2 cm)
#define POTHOLE_MIN_LEN_CM   10      // mai scurt = zgomot sau fisură
#define POTHOLE_MAX_LEN_CM   400     // mai lung = schimbare de nivel a drumului, nu groapă
#define POTHOLE_GAP_CM       15      // atâta drum fără adâncitură încheie groapa
#define BASELINE_ALPHA       0.005f  // cât de repede învață fiecare senzor nivelul drumului

// --- Geometria autobuzului (pentru poziția corectă a gropii) ---
// Antena GPS e în față / pe acoperiș; bara și roțile sunt în spate. Poziția GPS se mută înapoi
// cu aceste distanțe, altfel la un autobuz de 12 m groapa ar apărea cu metri mai în față.
#define ANTENNA_TO_BAR_M         10.5f
#define ANTENNA_TO_REAR_AXLE_M    7.5f

// --- Receptor GNSS generic (NMEA pe UART), de ex. u-blox NEO-M8N cu antenă externă ---
#define PIN_GPS_RX     16   // ESP32 RX2  <- TX receptor
#define PIN_GPS_TX     17   // ESP32 TX2  -> RX receptor
#define GPS_BAUD       9600

// --- Accelerometru (MPU6050, pe aceeași magistrală I2C) ---
// Simte gropile pe care LiDAR-ul nu le poate măsura (pline cu apă, în afara barei).
#define SAMPLE_PERIOD_MS     10      // 100 Hz
#define SHOCK_THRESHOLD_G    0.45f
#define EVENT_WINDOW_MS      300
#define REFRACTORY_MS        1000
#define SHOCK_HOLD_MS        1500    // așteptăm atât să vedem dacă LiDAR-ul a măsurat aceeași groapă
#define GRAVITY_EMA_ALPHA    0.01f

#define MIN_SPEED_KMH        8.0f    // în stații și la semafor nu măsurăm

// --- Alți pini ---
#define PIN_I2C_SDA    21
#define PIN_I2C_SCL    22
#define PIN_SD_CS       5   // SD: SCK 18, MISO 19, MOSI 23
#define PIN_BUTTON     27   // buton de service: apăsare lungă = trimite acum
#define PIN_LED         2
