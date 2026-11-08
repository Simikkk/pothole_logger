/*
  Configurare TFmini-S – se rulează O SINGURĂ DATĂ pentru fiecare senzor, înainte de montaj.

  Din fabrică, TFmini-S vorbește pe UART. Bara folosește 5 senzori pe aceeași magistrală I2C,
  deci fiecare trebuie trecut pe I2C și primește o adresă diferită (0x10 ... 0x14).

  Pași:
   1. Conectează UN singur senzor la ESP32 pe UART:
        TFmini-S TX (verde)  -> ESP32 GPIO 26
        TFmini-S RX (alb)    -> ESP32 GPIO 25
        5V (roșu), GND (negru)
   2. Pune adresa dorită la NEW_ADDRESS, încarcă sketch-ul, deschide Serial Monitor (115200)
      și verifică mesajele „ok”.
   3. Notează adresa pe senzor (cu marker), apoi repetă cu următorul senzor și altă adresă.
   4. După configurare, senzorul NU mai răspunde pe UART, doar pe I2C (SDA/SCL pe aceleași fire).

  Comenzile urmează manualul Benewake TFmini-S (cadru: 0x5A, lungime, comandă, date, sumă).
  Verifică-le în versiunea manualului pentru firmware-ul senzorului tău înainte de folosire.
*/

#define NEW_ADDRESS   0x10     // 0x10, 0x11, 0x12, 0x13, 0x14 – de la stânga la dreapta pe bară
#define FRAME_RATE_HZ 200      // trebuie să fie >= LIDAR_HZ din config.h

HardwareSerial tf(1);

// Trimite o comandă și adaugă suma de control (suma tuturor octeților, ultimii 8 biți)
void sendCommand(const uint8_t *body, uint8_t len, const char *what) {
  uint8_t sum = 0;
  for (uint8_t i = 0; i < len; i++) { tf.write(body[i]); sum += body[i]; }
  tf.write(sum);
  delay(100);
  int got = 0;
  while (tf.available()) { tf.read(); got++; }       // senzorul confirmă cu un răspuns scurt
  Serial.printf("  %-28s %s\n", what, got ? "ok" : "fara raspuns (verifica firele)");
}

void setup() {
  Serial.begin(115200);
  tf.begin(115200, SERIAL_8N1, 26, 25);
  delay(1500);
  Serial.printf("\nConfigurez TFmini-S: adresa I2C 0x%02X, %d Hz\n", NEW_ADDRESS, FRAME_RATE_HZ);

  const uint8_t rate[] = {0x5A, 0x06, 0x03, (uint8_t)(FRAME_RATE_HZ & 0xFF), (uint8_t)(FRAME_RATE_HZ >> 8)};
  const uint8_t addr[] = {0x5A, 0x05, 0x0B, NEW_ADDRESS};
  const uint8_t i2c[]  = {0x5A, 0x05, 0x0A, 0x01};
  const uint8_t save[] = {0x5A, 0x04, 0x11};

  sendCommand(rate, sizeof(rate), "frecventa de masurare");
  sendCommand(addr, sizeof(addr), "adresa I2C");
  sendCommand(i2c,  sizeof(i2c),  "trecere pe I2C");
  sendCommand(save, sizeof(save), "salvare in memorie");
  Serial.println("Gata. Deconecteaza senzorul, noteaza adresa pe el si treci la urmatorul.");
}

void loop() {}
