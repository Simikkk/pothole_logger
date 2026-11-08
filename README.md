# Harta gropilor

Sistem IoT pentru primărie: o bară cu senzori LiDAR montată pe autobuzele orașului măsoară
gropile din carosabil (adâncime, lungime, lățime) la fiecare trecere, iar datele ajung automat,
prin WiFi-ul de bord al autobuzului, pe o hartă web cu raport pentru echipele de reparații.

```
firmware/pothole_logger/   cod ESP32 pentru bara de senzori (Arduino IDE)
firmware/tfmini_config/    configurarea o singură dată a fiecărui senzor LiDAR
server/app.py              server Flask + SQLite (API + pagina web)
server/clustering.py       grupare detecții, severitate după adâncime, stare, străzi, starea autobuzelor
server/static/index.html   harta (Leaflet + OpenStreetMap)
server/simulate.py         simulator: autobuze cu LiDAR pe linii din Brașov
server/tests/              teste unitare (35)
```

## Pornire rapidă (dublu-click)

| Fișier | Ce face |
|--------|---------|
| **DEMO.bat** | pornește harta cu 6 autobuze simulate – pentru prezentare, fără hardware |
| **PORNESTE.bat** | pornește serverul pentru datele reale de la autobuze |
| `porneste.sh` / `porneste.sh --demo` | același lucru pe Mac/Linux |

Liniile din demo urmează străzi reale din Brașov. La **prima** pornire a demo-ului e nevoie de
internet: traseele se descarcă de la serviciul public OSRM și se salvează în
`server/routes_cache.json`, apoi demo-ul merge și offline. Rulează DEMO.bat o dată înainte de prezentare.
Fereastra neagră arată și cât de bine a găsit sistemul gropile simulate.

Singura cerință e Python 3 (la instalare bifează *Add python.exe to PATH*).

## 1. Hardware – o singură carcasă pe autobuz

Bara se montează sub bara de protecție din spate, la ~40–50 cm de asfalt, prinsă **rigid**
(fără tampoane de cauciuc), cu senzorii orientați vertical în jos.

| Componentă | Rol | Cost orientativ* |
|------------|-----|------------------|
| ESP32 DevKit | procesare, WiFi către router-ul autobuzului | 35 lei |
| 5 × LiDAR TFmini-S (I2C) | distanța până la asfalt, 200 de măsurători/s fiecare | 5 × 180 lei |
| MPU6050 | gropile pe care LiDAR-ul nu le poate măsura (apă) | 15 lei |
| Receptor GNSS generic NMEA (ex. u-blox NEO-M8N) + antenă externă pe acoperiș | poziție, viteză, direcție | 80 lei |
| Convertor DC-DC auto 24 V → 5 V, min. 2 A, cu protecții | alimentare din autobuz | 50 lei |
| Modul microSD + card | coadă de date când nu e semnal | 20 lei |
| Carcasă IP67 cu ferestre separate pentru fiecare senzor | protecție la apă, noroi | 100–200 lei |

\* prețuri aproximative, de verificat. Total ~1.200–1.400 lei pe autobuz.

Conexiuni ESP32:

| Componentă | Pini |
|------------|------|
| TFmini-S × 5 + MPU6050 | toate pe I2C: SDA 21, SCL 22 (rezistențe pull-up 4,7 kΩ la 3,3 V). TFmini-S se alimentează la 5 V |
| Receptor GNSS | TX → 16, RX ← 17 |
| microSD | CS 5, SCK 18, MISO 19, MOSI 23 |
| Buton de service | 27 și GND |
| LED | cel de pe placă (2) |

Fiecare senzor TFmini-S trebuie trecut o dată pe I2C, cu adresă proprie (0x10 … 0x14, de la
stânga la dreapta): încarcă `firmware/tfmini_config/tfmini_config.ino` cu `NEW_ADDRESS` schimbat
pentru fiecare senzor (instrucțiuni în fișier).

**Limitele senzorilor, asumate:** TFmini-S e un senzor ieftin (precizie de ordinul centimetrilor).
Algoritmul compară fiecare senzor cu nivelul drumului din jur, ceea ce elimină erorile constante,
dar adâncimile sub 2 cm nu se pot măsura sigur. 5 senzori văd o fâșie de ~1 m din lățimea benzii;
autobuzele nu merg mereu exact pe același loc, deci în câteva treceri acoperă toată banda.

## 2. Firmware

1. Arduino IDE → plăcile **esp32 by Espressif** și biblioteca **TinyGPSPlus**.
2. Deschide `firmware/pothole_logger/pothole_logger.ino` și editează `config.h`:
   codul autobuzului, WiFi-ul de bord, adresa serverului, distanța antenă–bară.
3. Placă: *ESP32 Dev Module* → Upload. Serial Monitor (115200) arată la pornire dacă fiecare
   senzor LiDAR răspunde.

Cum măsoară o groapă:
1. Fiecare senzor măsoară distanța până la asfalt de 200 de ori pe secundă (la 50 km/h, o
   măsurătoare la ~7 cm) și își învață încet nivelul drumului.
2. Caroseria se leagănă pe suspensii și mută toți senzorii deodată. **Mediana** celor 5 abateri e
   exact această mișcare comună și se scade; o groapă e sub 1–2 senzori, deci nu strică mediana.
3. Ce rămâne peste 2 cm e adâncitură: durata × viteza = lungimea, numărul de senzori × 20 cm =
   lățimea minimă, a doua cea mai mare valoare = adâncimea (un vârf izolat e ignorat).
4. Poziția: ultimul fix GPS, dus înainte cu viteza × timpul scurs și înapoi cu distanța de la
   antenă la bară (un autobuz are 12 m).
5. Accelerometrul salvează un „șoc” doar dacă LiDAR-ul **nu** a măsurat aceeași groapă:
   de obicei e o groapă plină cu apă, în care laserul vede doar suprafața apei.

Algoritmul a fost rulat pe calculator cu senzori simulați: găsește o groapă de 5,5 cm × 60 cm
(măsoară 5,7 cm × 65 cm), nu dă alarme pe drum bun cu caroseria legănându-se ±2 cm și nu
raportează nimic fals pentru o groapă plină cu apă.

Trimiterea datelor: dispozitivul stă conectat la router-ul autobuzului și trimite detecțiile noi
o dată pe minut, iar la 5 minute un semn de viață cu starea fiecărui senzor (ce procent din
citiri au fost bune). Fără semnal, datele așteaptă pe card. Trimiterea rulează pe al doilea
nucleu al ESP32, deci detecția nu se oprește niciodată.
Pe autobuz: **rețea WiFi separată** de cea a pasagerilor, `https://` și cheie API
(certificatul rădăcină al serverului se pune în `ROOT_CA`, instrucțiuni în `config.h`).

LED de stare: clipește o dată pe secundă = caută GPS · clipire scurtă la 2 s = fără WiFi ·
stins = totul funcționează (se aprinde scurt la fiecare groapă).
Buton de service: apăsare lungă (2 s) = trimite acum.

**Server public cu HTTPS**: serverul Flask din proiect e pentru rețeaua locală. Pentru autobuze
reale, rulează-l pe un server cu nume de domeniu, în spatele unui proxy care face HTTPS automat
(de exemplu Caddy, cu certificat gratuit Let's Encrypt), și setează `POTHOLE_API_KEY`.

## 3. Server

```bash
cd server
pip install -r requirements.txt
python app.py                 # http://localhost:5000
python simulate.py            # într-un alt terminal: 6 autobuze simulate
python -m unittest discover -s tests
```

O bază de date din versiunea veche (doar accelerometru) e migrată automat la pornire.

Pe server:
- detecțiile aflate la sub 12 m una de alta formează o groapă, **confirmată** după 2 treceri separate;
- **severitatea după adâncime**: sub 3 cm mică, 3–6 cm medie, peste 6 cm mare
  (praguri de stabilit cu primăria, în `clustering.py`);
- **asfalt estimat** = adâncime × lungime × lățime × 2,4 kg/L, pentru planificarea reparațiilor;
- o groapă simțită doar de accelerometru apare ca **nemăsurată (posibil apă)** până la o trecere pe uscat.

## 4. Funcții ale hărții

- **Gropi colorate după adâncime**, cu dimensiunile și asfaltul estimat în popup.
- **Starea gropii**: *Am raportat-o* / *E reparată*. Gropile reparate ies din raport; dacă un
  autobuz o detectează din nou, revine automat ca „semnalată”, cu mențiunea „reapărută”.
- **Străzi cu cele mai multe gropi**, cu numele de la OpenStreetMap (Nominatim), salvate în baza
  de date. Fără internet, lista apare „în căutare”. Dezactivare: `POTHOLE_STREETS=0`.
- **Autobuze**: starea fiecărei bare de senzori – funcționează, senzor murdar (sub 50% citiri
  bune), fără GPS sau offline (fără semn de viață de 15 minute).
- **Raport CSV** pentru primărie (se deschide direct în Excel).

Confidențialitate: datele vin doar de la autobuzele flotei; senzorii privesc în jos, spre asfalt
(nu există camere); nu se salvează trasee, doar locurile cu gropi; serverul nu salvează adrese IP.

## 5. Evaluare pentru proiect

- **Acuratețe**: simulatorul compară gropile confirmate pe hartă cu gropile „reale” generate și
  afișează câte au fost găsite, câte confirmări sunt false și eroarea medie a adâncimii.
  Pe teren: o echipă măsoară cu rigla câteva gropi de pe o linie și compară cu harta.
- **Utilizabilitate (HCI)**: 5–8 persoane, sarcini concrete („găsește cea mai adâncă groapă din
  ultima săptămână”, „care autobuz are un senzor murdar?”, „descarcă raportul pentru Strada X”),
  timp, greșeli, apoi chestionarul **SUS** (System Usability Scale).

## API

| Metodă | Endpoint | Descriere |
|--------|----------|-----------|
| POST | `/api/events` | `{"device_id", "events": [{"t", "lat", "lon", "v", "k": "lidar", "d", "l", "w"} sau {…, "k": "soc", "g"}]}` – duplicatele sunt ignorate |
| POST | `/api/heartbeat` | `{"device_id", "sensors": [{"valid": 0.97}, …], "gps", "pending", "fw"}` |
| GET  | `/api/devices` | starea autobuzelor |
| GET  | `/api/potholes?days=N` | gropile grupate, cu dimensiuni |
| GET  | `/api/stats?days=N` | statistici, inclusiv asfaltul estimat |
| POST | `/api/potholes/status` | `{"lat", "lon", "status": "semnalata" / "raportata" / "reparata"}` |
| GET  | `/api/streets?days=N` | străzile cu cele mai multe gropi confirmate și nereparate |
| GET  | `/api/report.csv?days=N&confirmed=1&fixed=1` | raport pentru primărie |
| DELETE | `/api/devices/<cod>` | șterge toate datele unui autobuz (de ex. la scoaterea din flotă) |
