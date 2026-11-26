/*
  Taupunkt-Lüftersteuerung mit Webseite
  Board:    ESP32-C3 (SuperMini / DevKitM), "USB CDC On Boot: Enabled"
  Sensoren: 2x SHT31-Sonde (FS400-SHT3X), beide fest auf Adresse 0x44
            -> jede Sonde an eigenen Pins, der I2C-Bus wird vor jeder
               Messung auf die Pins der jeweiligen Sonde umgeschaltet
  Ausgang:  Relaismodul für 230-V-Lüfter
  Upload:   per USB oder über WLAN (OTA, Passwort OTA_PASS in wlan_daten.h)
  Web:      http://taupunkt.local  (oder IP aus dem seriellen Monitor)
            Aktuelle Werte, Verlauf 24 h / 7 Tage, Schwellen einstellbar
  Verlauf:  wird im Flash (LittleFS) gesichert und übersteht Neustarts;
            die Uhrzeit (NTP) ordnet eine Lücke nach dem Neustart richtig ein

  WLAN-Zugangsdaten in wlan_daten.h eintragen.
  Keine externe Bibliothek nötig.

  Logik:
    Messwerte werden gleitend gemittelt (Zeitkonstante GLAETTUNG_S).
    Ein   : Td_innen - Td_außen > diffEin  UND Tinnen > tInnenMin UND RHinnen > rhInnenMin
            UND Taußen >= tAussenMin UND keine Zwangspause
    Aus   : Differenz < diffAus ODER Tinnen < tInnenMin ODER RHinnen < rhInnenMin
    Mindestlaufzeit und Mindestpause je MIN_ZEIT_MS
    Sofort AUS (Vorrang vor Mindestzeiten):
      Sensorfehler / unplausible Werte, Frostschutz (Taußen < tAussenMin)
    Optional: nach maxLaufMin Laufzeit Zwangspause von pauseMin (maxLaufMin = 0: aus)
    Die Regelung läuft unabhängig davon, ob WLAN verbunden ist.
*/

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <time.h>
#include "wlan_daten.h"

// ---------------- Konfiguration ----------------
const int  SDA_INNEN  = 9, SCL_INNEN  = 8;   // Sonde innen
const int  SDA_AUSSEN = 7, SCL_AUSSEN = 6;   // Sonde außen
const int  RELAY_PIN  = 4;                   // kein Strapping-Pin am C3
const bool RELAY_ACTIVE_LOW = false;         // true, wenn dein Modul active LOW ist

const char *HOSTNAME = "taupunkt";           // -> http://taupunkt.local

const uint8_t  SHT_ADDR = 0x44;
const uint32_t I2C_TAKT = 50000;             // langsamer Takt wegen 1,5 m Kabel

// Standardwerte, über die Webseite änderbar (werden im Flash gespeichert)
float diffEin    = 5.0;    // K
float diffAus    = 2.0;    // K (Hysterese)
float tInnenMin  = 10.0;   // °C, darunter nicht lüften (Auskühlen)
float rhInnenMin = 50.0;   // %, darunter gibt es nichts zu trocknen
float tAussenMin = -5.0;   // °C, Frostschutz: darunter nicht lüften
float maxLaufMin = 0;      // min, maximale Laufzeit am Stück (0 = unbegrenzt)
float pauseMin   = 10;     // min, Pause nach Erreichen der maximalen Laufzeit

const unsigned long MESS_INTERVALL_MS = 5000;                 // alle 5 s messen
const unsigned long MIN_ZEIT_MS       = 5UL * 60UL * 1000UL;  // 5 min Mindestlauf/-pause
const float GLAETTUNG_S = 120.0;   // Zeitkonstante des gleitenden Mittels (s)

// Korrekturwerte (Offset) falls Sensoren abweichen
const float T_OFFSET_INNEN = 0.0, RH_OFFSET_INNEN = 0.0;
const float T_OFFSET_AUSSEN = 0.0, RH_OFFSET_AUSSEN = 0.0;

// Verlauf: kurz 288 Punkte × 5 min = 24 h (Momentwerte),
//          lang 336 Punkte × 30 min = 7 Tage (Mittelwerte)
const int KURZ_N = 288, LANG_N = 336;
const uint32_t KURZ_IV_S = 5UL * 60UL, LANG_IV_S = 30UL * 60UL;
const char *VERLAUF_DATEI = "/verlauf.bin";
const char *VERLAUF_TEMP  = "/verlauf.tmp";
// -----------------------------------------------

WebServer server(80);
Preferences prefs;

bool fanOn = false;
unsigned long letzterWechsel = 0;   // für Mindestzeiten
unsigned long zustandSeit = 0;      // für Anzeige "seit ..."
unsigned long letzteMessung = 0;
bool mdnsGestartet = false;
unsigned long wlanWegSeit = 0;      // 0 = verbunden
int wlanVersuche = 0;
const unsigned long WLAN_NEU_MS = 30000;   // alle 30 s neu verbinden, solange getrennt
// Sendeleistung: 8,5 dBm hilft bei manchen SuperMini-Boards gegen Verbindungsprobleme,
// bei schwachem Empfang hört der Router den ESP32 dann aber nicht mehr.
const wifi_power_t WLAN_SENDELEISTUNG = WIFI_POWER_15dBm;

// Zwangspause nach maximaler Laufzeit
bool pauseAktiv = false;
unsigned long pauseStart = 0;

// aktuelle (geglättete) Messwerte
float mTIn = NAN, mRhIn = NAN, mTOut = NAN, mRhOut = NAN, mDIn = NAN, mDOut = NAN;
bool sensorOk = false;

// Verlauf (Werte x10 als int16, INT16_MIN = kein Wert;
// fan = Anteil der Zeit mit Lüfter an in %, LUEFTER_UNBEKANNT = kein Wert)
const uint8_t LUEFTER_UNBEKANNT = 255;
struct Punkt { int16_t tIn, rhIn, tOut, rhOut, dIn, dOut; uint8_t fan; };
struct Puffer {
  Punkt *p;
  int n;
  uint32_t ivS;              // Sekunden pro Punkt
  int kopf, anzahl;
  uint32_t letzteZeit;       // Unixzeit des neuesten Punkts (0 = unbekannt)
  unsigned long letzterMs;   // millis() des neuesten Punkts
};
Punkt kurzP[KURZ_N], langP[LANG_N];
Puffer kurz = {kurzP, KURZ_N, KURZ_IV_S, 0, 0, 0, 0};
Puffer lang = {langP, LANG_N, LANG_IV_S, 0, 0, 0, 0};
// Summen für den 30-min-Mittelwert
struct Summe { float tIn, rhIn, tOut, rhOut; int nIn, nOut, n, fan; } summe = {};
bool dateisystemOk = false;
bool verlaufBereit = false;   // erst aufzeichnen, wenn die Lücke seit dem Neustart bekannt ist

// ================= Sensoren & Regelung =================

void setRelay(bool on) {
  digitalWrite(RELAY_PIN, (on != RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

uint8_t crc8(const uint8_t *d, int len) {   // Sensirion CRC (Poly 0x31, Init 0xFF)
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= d[i];
    for (int b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

// Liest eine SHT31 an den angegebenen Pins. false bei Fehler.
bool leseSht(int sda, int scl, float &t, float &rh) {
  t = rh = NAN;
  Wire.end();
  Wire.begin(sda, scl, I2C_TAKT);
  Wire.setTimeOut(50);

  Wire.beginTransmission(SHT_ADDR);
  Wire.write(0x24);   // Single Shot, hohe Wiederholgenauigkeit,
  Wire.write(0x00);   // ohne Clock Stretching
  if (Wire.endTransmission() != 0) return false;

  delay(20);          // Messzeit max. 15,5 ms

  uint8_t d[6];
  if (Wire.requestFrom(SHT_ADDR, (uint8_t)6) != 6) return false;
  for (int i = 0; i < 6; i++) d[i] = Wire.read();
  if (crc8(d, 2) != d[2] || crc8(d + 3, 2) != d[5]) return false;

  uint16_t rawT  = (d[0] << 8) | d[1];
  uint16_t rawRH = (d[3] << 8) | d[4];
  t  = -45.0 + 175.0 * rawT / 65535.0;
  rh = 100.0 * rawRH / 65535.0;
  return true;
}

float taupunkt(float t, float rh) {
  const float a = 17.62, b = 243.12;  // Magnus-Formel
  rh = max(rh, 0.1f);                 // log(0) vermeiden
  float g = log(rh / 100.0) + a * t / (b + t);
  return b * g / (a - g);
}

bool plausibel(float t, float rh) {
  return !isnan(t) && !isnan(rh) && t > -40.0 && t < 80.0 && rh >= 0.0 && rh <= 100.0;
}

void schalte(bool on, unsigned long jetzt) {
  if (on != fanOn) {
    fanOn = on;
    letzterWechsel = jetzt;
    zustandSeit = jetzt;
  }
  setRelay(fanOn);
}

// Exponentieller gleitender Mittelwert; startet beim ersten gültigen Wert neu
float glaette(float alt, float neu) {
  if (isnan(alt)) return neu;
  const float k = (MESS_INTERVALL_MS / 1000.0f) / (GLAETTUNG_S + MESS_INTERVALL_MS / 1000.0f);
  return alt + k * (neu - alt);
}

unsigned long minutenMs(float m) { return (unsigned long)(m * 60000.0f); }

int16_t kodiere(float v) { return isnan(v) ? INT16_MIN : (int16_t)lroundf(v * 10.0f); }

// Unixzeit, sobald per NTP gestellt, sonst 0
uint32_t unixZeit() {
  time_t t = time(nullptr);
  return t > 1700000000 ? (uint32_t)t : 0;
}

Punkt punktAus(float tIn, float rhIn, float tOut, float rhOut, uint8_t fan) {
  Punkt q;
  q.tIn = kodiere(tIn);   q.rhIn = kodiere(rhIn);
  q.tOut = kodiere(tOut); q.rhOut = kodiere(rhOut);
  q.dIn  = kodiere(isnan(tIn)  || isnan(rhIn)  ? NAN : taupunkt(tIn, rhIn));
  q.dOut = kodiere(isnan(tOut) || isnan(rhOut) ? NAN : taupunkt(tOut, rhOut));
  q.fan = fan;
  return q;
}

void einreihen(Puffer &b, const Punkt &q) {
  b.p[b.kopf] = q;
  b.kopf = (b.kopf + 1) % b.n;
  if (b.anzahl < b.n) b.anzahl++;
}

void anhaengen(Puffer &b, const Punkt &q, unsigned long jetzt) {
  einreihen(b, q);
  b.letzterMs = jetzt;
  b.letzteZeit = unixZeit();
}

// ---- Sicherung im Flash: Kopf + beide Puffer, Schreiben über Temp-Datei ----
const uint32_t VERLAUF_MAGIE = 0x54505631;   // "TPV1"
struct DateiKopf {
  uint32_t magie;
  uint16_t kurzN, langN, punktGroesse;
  int16_t kurzKopf, kurzAnzahl, langKopf, langAnzahl;
  uint32_t kurzZeit, langZeit;
};

void verlaufSichern() {
  if (!dateisystemOk) return;
  DateiKopf k = {VERLAUF_MAGIE, KURZ_N, LANG_N, sizeof(Punkt),
                 (int16_t)kurz.kopf, (int16_t)kurz.anzahl, (int16_t)lang.kopf, (int16_t)lang.anzahl,
                 kurz.letzteZeit, lang.letzteZeit};
  File f = LittleFS.open(VERLAUF_TEMP, "w");
  if (!f) return;
  bool ok = f.write((const uint8_t *)&k, sizeof k) == sizeof k &&
            f.write((const uint8_t *)kurzP, sizeof kurzP) == sizeof kurzP &&
            f.write((const uint8_t *)langP, sizeof langP) == sizeof langP;
  f.close();
  if (ok) ok = LittleFS.rename(VERLAUF_TEMP, VERLAUF_DATEI);
  if (!ok) Serial.println("Verlauf konnte nicht gesichert werden");
}

void verlaufLaden() {
  File f = LittleFS.open(VERLAUF_DATEI, "r");
  if (!f) return;
  DateiKopf k;
  bool ok = f.read((uint8_t *)&k, sizeof k) == sizeof k &&
            k.magie == VERLAUF_MAGIE && k.kurzN == KURZ_N && k.langN == LANG_N &&
            k.punktGroesse == sizeof(Punkt) &&
            k.kurzKopf >= 0 && k.kurzKopf < KURZ_N && k.kurzAnzahl >= 0 && k.kurzAnzahl <= KURZ_N &&
            k.langKopf >= 0 && k.langKopf < LANG_N && k.langAnzahl >= 0 && k.langAnzahl <= LANG_N &&
            f.read((uint8_t *)kurzP, sizeof kurzP) == sizeof kurzP &&
            f.read((uint8_t *)langP, sizeof langP) == sizeof langP;
  f.close();
  if (!ok) { Serial.println("Gesicherter Verlauf ungültig, starte leer"); return; }
  kurz.kopf = k.kurzKopf; kurz.anzahl = k.kurzAnzahl; kurz.letzteZeit = k.kurzZeit;
  lang.kopf = k.langKopf; lang.anzahl = k.langAnzahl; lang.letzteZeit = k.langZeit;
  Serial.printf("Verlauf geladen: %d Punkte (24 h), %d Punkte (7 Tage)\n", kurz.anzahl, lang.anzahl);
}

// Zeitraster des Puffers über den Neustart fortsetzen: verpasste Punkte als Lücke
// eintragen, nächster Punkt zur nächsten regulären Zeit.
void lueckeFuellen(Puffer &b, uint32_t t, unsigned long jetzt) {
  if (b.anzahl == 0 || b.letzteZeit == 0 || t < b.letzteZeit) return;
  uint32_t luecke = t - b.letzteZeit;
  uint32_t fehlend = luecke / b.ivS;
  if (fehlend >= (uint32_t)b.n) { b.kopf = b.anzahl = 0; return; }   // alles veraltet
  Punkt leer = punktAus(NAN, NAN, NAN, NAN, LUEFTER_UNBEKANNT);
  for (uint32_t i = 0; i < fehlend; i++) einreihen(b, leer);
  b.letzterMs = jetzt - (luecke % b.ivS) * 1000UL;
}

// Wartet nach dem Start bis zu 3 min auf die Uhrzeit, damit die Lücke bekannt ist
void verlaufVorbereiten(unsigned long jetzt) {
  if (verlaufBereit) return;
  uint32_t t = unixZeit();
  if (t == 0 && jetzt < 180000UL) return;
  verlaufBereit = true;
  if (t) {
    lueckeFuellen(kurz, t, jetzt);
    lueckeFuellen(lang, t, jetzt);
  }
}

// Bei jeder Messung: Mittelwerte sammeln, fällige Punkte anhängen, dann sichern
void verlaufFortschreiben(unsigned long jetzt) {
  summe.n++;
  if (fanOn) summe.fan++;
  if (!isnan(mTIn))  { summe.tIn  += mTIn;  summe.rhIn  += mRhIn;  summe.nIn++; }
  if (!isnan(mTOut)) { summe.tOut += mTOut; summe.rhOut += mRhOut; summe.nOut++; }

  verlaufVorbereiten(jetzt);
  if (!verlaufBereit) return;

  bool neu = false;
  if (jetzt - kurz.letzterMs >= KURZ_IV_S * 1000UL) {
    anhaengen(kurz, punktAus(mTIn, mRhIn, mTOut, mRhOut, fanOn ? 100 : 0), jetzt);
    neu = true;
  }
  if (jetzt - lang.letzterMs >= LANG_IV_S * 1000UL) {
    anhaengen(lang, punktAus(summe.nIn  ? summe.tIn  / summe.nIn  : NAN,
                             summe.nIn  ? summe.rhIn / summe.nIn  : NAN,
                             summe.nOut ? summe.tOut / summe.nOut : NAN,
                             summe.nOut ? summe.rhOut / summe.nOut : NAN,
                             (uint8_t)lroundf(100.0f * summe.fan / summe.n)), jetzt);
    summe = {};
    neu = true;
  }
  if (neu) verlaufSichern();
}

void messen(unsigned long jetzt) {
  float tIn, rhIn, tOut, rhOut;
  bool okIn  = leseSht(SDA_INNEN,  SCL_INNEN,  tIn,  rhIn);
  bool okOut = leseSht(SDA_AUSSEN, SCL_AUSSEN, tOut, rhOut);

  tIn  += T_OFFSET_INNEN;   rhIn  = constrain(rhIn  + RH_OFFSET_INNEN,  0.0, 100.0);
  tOut += T_OFFSET_AUSSEN;  rhOut = constrain(rhOut + RH_OFFSET_AUSSEN, 0.0, 100.0);

  okIn  = okIn  && plausibel(tIn, rhIn);
  okOut = okOut && plausibel(tOut, rhOut);
  sensorOk = okIn && okOut;

  // Gleitender Mittelwert gegen Messrauschen (Wind, Regen, Sonne am Sensor).
  // Nach einem Fehler startet die Mittelung des betroffenen Sensors neu.
  if (okIn)  { mTIn  = glaette(mTIn, tIn);   mRhIn  = glaette(mRhIn, rhIn); }
  else       { mTIn  = mRhIn  = NAN; }
  if (okOut) { mTOut = glaette(mTOut, tOut); mRhOut = glaette(mRhOut, rhOut); }
  else       { mTOut = mRhOut = NAN; }

  if (maxLaufMin <= 0 || (pauseAktiv && jetzt - pauseStart >= minutenMs(pauseMin))) pauseAktiv = false;

  if (!sensorOk) {
    // ---- Sensorfehler: Lüfter sofort aus ----
    mDIn = mDOut = NAN;
    Serial.printf("Sensorfehler (innen %s, aussen %s) -> Luefter AUS\n",
                  okIn ? "OK" : "FEHLER", okOut ? "OK" : "FEHLER");
    schalte(false, jetzt);
  } else {
    mDIn  = taupunkt(mTIn, mRhIn);
    mDOut = taupunkt(mTOut, mRhOut);
    float diff = mDIn - mDOut;

    bool zeitOk = (jetzt - letzterWechsel) >= MIN_ZEIT_MS;
    bool frost  = mTOut < tAussenMin;
    bool laufzeitUm = fanOn && maxLaufMin > 0 && (jetzt - letzterWechsel) >= minutenMs(maxLaufMin);

    if (fanOn && frost) {
      schalte(false, jetzt);                 // Frostschutz: sofort aus
    } else if (laufzeitUm) {
      schalte(false, jetzt);                 // maximale Laufzeit erreicht -> Pause
      pauseAktiv = true;
      pauseStart = jetzt;
    } else if (!fanOn && zeitOk && !pauseAktiv && !frost &&
               diff > diffEin && mTIn > tInnenMin && mRhIn > rhInnenMin) {
      schalte(true, jetzt);
    } else if (fanOn && zeitOk && ((diff < diffAus) || (mTIn < tInnenMin) || (mRhIn < rhInnenMin))) {
      schalte(false, jetzt);
    } else {
      setRelay(fanOn);
    }

    Serial.printf("In: %.1f C %.1f %% Td %.1f | Out: %.1f C %.1f %% Td %.1f | Diff %.1f K | Luefter %s%s\n",
                  mTIn, mRhIn, mDIn, mTOut, mRhOut, mDOut, diff, fanOn ? "EIN" : "AUS",
                  pauseAktiv ? " (Pause)" : (frost ? " (Frostschutz)" : ""));
  }

  verlaufFortschreiben(jetzt);
}

// ================= Webseite =================

// Favicon: Lüfterblatt
const char FAVICON[] PROGMEM =
  "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 24 24\">"
  "<path id=\"b\" fill=\"#2563eb\" d=\"M12 10C9 8 9.3 2.2 12.8 2c3.2.2 2.4 5.6-.8 8z\"/>"
  "<use href=\"#b\" transform=\"rotate(90 12 12)\"/>"
  "<use href=\"#b\" transform=\"rotate(180 12 12)\"/>"
  "<use href=\"#b\" transform=\"rotate(270 12 12)\"/>"
  "<circle cx=\"12\" cy=\"12\" r=\"2.2\" fill=\"#2563eb\"/></svg>";

const char SEITE[] PROGMEM = R"rawliteral(<!doctype html>
<html lang="de"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Taupunkt-Lüfter</title>
<link rel="icon" type="image/svg+xml" href="/favicon.svg">
<style>
:root{--bg:#f4f5f7;--card:#fff;--tx:#1b1f24;--mu:#6b7280;--bd:#e3e5e8;--in:#2563eb;--out:#d97706;--on:#16a34a;--off:#9ca3af;--err:#dc2626}
@media(prefers-color-scheme:dark){:root{--bg:#111418;--card:#1a1e23;--tx:#e8eaed;--mu:#9aa0a6;--bd:#2a2f36;--in:#60a5fa;--out:#fbbf24;--on:#4ade80;--off:#6b7280;--err:#f87171}}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--tx)}
main{max-width:720px;margin:0 auto;padding:16px}
h1{font-size:1.25rem;margin:4px 0 14px}
.card{background:var(--card);border:1px solid var(--bd);border-radius:12px;padding:14px 16px;margin-bottom:12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:12px;margin-bottom:12px}
.grid .card{margin:0}
.lbl{font-size:.75rem;color:var(--mu);text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px}
.big{font-size:2rem;font-weight:650;font-variant-numeric:tabular-nums;line-height:1.1}
.row{display:flex;justify-content:space-between;font-variant-numeric:tabular-nums;margin-top:6px}
.mu{color:var(--mu);font-size:.85rem}
.status{display:flex;align-items:center;gap:14px;flex-wrap:wrap}
.badge{padding:6px 14px;border-radius:999px;font-weight:650;color:#fff;background:var(--off)}
.err{color:var(--err)}
canvas{width:100%;height:220px;display:block}
.leg span{margin-right:12px}
.chk{display:flex;justify-content:space-between;align-items:center;gap:8px;flex-wrap:wrap;margin-bottom:6px}
.chk .lbl{margin:0}
.seg{display:flex;gap:4px;flex-wrap:wrap}
.seg button{grid-column:auto;margin:0;padding:4px 10px;border:1px solid var(--bd);border-radius:999px;background:transparent;color:var(--mu);font-size:.8rem}
.seg button.akt{background:var(--in);border-color:var(--in);color:#fff}
form{display:grid;grid-template-columns:1fr 96px;gap:8px 12px;align-items:center}
input{width:100%;padding:7px 8px;border:1px solid var(--bd);border-radius:8px;background:var(--bg);color:var(--tx);font-size:1rem}
button{grid-column:1/-1;margin-top:4px;padding:10px;border:0;border-radius:8px;background:var(--in);color:#fff;font-size:1rem;cursor:pointer}
</style></head><body><main>
<h1>Taupunkt-Lüfter</h1>

<div class="card status">
  <span class="badge" id="fan">–</span>
  <div><div id="diff" class="big" style="font-size:1.4rem">–</div><div class="mu" id="seit"></div></div>
</div>

<div class="grid">
  <div class="card"><div class="lbl" style="color:var(--in)">Innen</div>
    <div class="big" id="dIn">–</div><div class="mu">Taupunkt</div>
    <div class="row"><span>Temperatur</span><span id="tIn">–</span></div>
    <div class="row"><span>Luftfeuchte</span><span id="rhIn">–</span></div>
    <div class="row"><span>abs. Feuchte</span><span id="ahIn">–</span></div></div>
  <div class="card"><div class="lbl" style="color:var(--out)">Außen</div>
    <div class="big" id="dOut">–</div><div class="mu">Taupunkt</div>
    <div class="row"><span>Temperatur</span><span id="tOut">–</span></div>
    <div class="row"><span>Luftfeuchte</span><span id="rhOut">–</span></div>
    <div class="row"><span>abs. Feuchte</span><span id="ahOut">–</span></div></div>
</div>

<div class="card"><div class="chk"><div class="lbl" id="chT">Taupunkt – Verlauf</div>
  <div class="seg" id="ber"><button data-b="24h">24 h</button><button data-b="7d">7 Tage</button></div>
  <div class="seg" id="seg"><button data-a="tp">Taupunkt</button><button data-a="t">Temperatur</button><button data-a="rh">Rel. Feuchte</button><button data-a="ah">Abs. Feuchte</button></div></div>
  <canvas id="ch"></canvas>
  <div class="mu leg"><span style="color:var(--in)">━ innen</span><span style="color:var(--out)">━ außen</span><span style="color:var(--on)">▮ Lüfter an</span></div>
</div>

<div class="card"><div class="lbl">Einstellungen</div>
  <form id="f">
    <label for="diffEin">Einschalten ab Taupunkt-Differenz (K)</label><input id="diffEin" name="diffEin" type="number" step="0.5" min="0.5" max="20">
    <label for="diffAus">Ausschalten unter Differenz (K)</label><input id="diffAus" name="diffAus" type="number" step="0.5" min="0" max="19.5">
    <label for="tMin">Innen-Temperatur mindestens (°C)</label><input id="tMin" name="tMin" type="number" step="0.5" min="-10" max="30">
    <label for="rhMin">Innen-Feuchte mindestens (%)</label><input id="rhMin" name="rhMin" type="number" step="1" min="0" max="95">
    <label for="tAMin">Außen-Temperatur mindestens, Frostschutz (°C)</label><input id="tAMin" name="tAMin" type="number" step="0.5" min="-30" max="15">
    <label for="maxLauf">Maximale Laufzeit am Stück (min, 0 = unbegrenzt)</label><input id="maxLauf" name="maxLauf" type="number" step="1" min="0" max="240">
    <label for="pause">Pause danach (min)</label><input id="pause" name="pause" type="number" step="1" min="5" max="240">
    <button>Speichern</button>
  </form>
  <div class="mu" id="fm" style="margin-top:8px"></div>
</div>
<div class="mu" id="info"></div>
</main>
<script>
const $=id=>document.getElementById(id);
const zf=(v,n)=>v.toLocaleString('de-AT',{minimumFractionDigits:n,maximumFractionDigits:n});
const f1=v=>v==null?'–':zf(v,1);
// absolute Feuchte in g/m³ (Magnus-Formel, gleiche Konstanten wie im ESP32)
const absF=(t,rh)=>t==null||rh==null?null:6.112*Math.exp(17.62*t/(243.12+t))*rh*2.1674/(273.15+t);
const f2=v=>v==null?'–':zf(v,1)+' g/m³';
function dauer(s){if(s<3600)return Math.floor(s/60)+' min';if(s<86400)return zf(s/3600,1)+' h';return zf(s/86400,1)+' Tage';}
function grund(d){
  if(d.pauseRest>0)return 'Pause nach max. Laufzeit, noch '+dauer(d.pauseRest+59);
  if(d.tOut<=d.tAMin)return 'Frostschutz';
  if(d.tIn<=d.tMin)return 'innen zu kalt';
  if(d.rhIn<=d.rhMin)return 'innen trocken genug';
  if(d.dIn-d.dOut<=d.diffEin)return 'Differenz unter Einschaltschwelle';
  return 'Mindestpause';
}
let formGeladen=false;

async function daten(){
  try{
    const d=await (await fetch('/daten')).json();
    $('dIn').textContent=f1(d.dIn)+' °C';  $('dOut').textContent=f1(d.dOut)+' °C';
    $('tIn').textContent=f1(d.tIn)+' °C';  $('tOut').textContent=f1(d.tOut)+' °C';
    $('rhIn').textContent=f1(d.rhIn)+' %'; $('rhOut').textContent=f1(d.rhOut)+' %';
    $('ahIn').textContent=f2(absF(d.tIn,d.rhIn)); $('ahOut').textContent=f2(absF(d.tOut,d.rhOut));
    const b=$('fan'); b.textContent=d.fan?'Lüfter EIN':'Lüfter AUS';
    b.style.background=d.fan?'var(--on)':'var(--off)';
    const df=$('diff');
    if(d.ok){df.textContent='Differenz '+f1(d.dIn-d.dOut)+' K';df.className='big';}
    else{df.textContent='Sensorfehler';df.className='big err';}
    df.style.fontSize='1.4rem';
    $('seit').textContent=(d.fan?'an':'aus')+' seit '+dauer(d.seit)+(!d.fan&&d.ok?' · '+grund(d):'');
    if(!formGeladen){['diffEin','diffAus','tMin','rhMin','tAMin','maxLauf','pause'].forEach(k=>$(k).value=d[k]);formGeladen=true;}
    $('info').textContent='WLAN '+d.rssi+' dBm · in Betrieb seit '+dauer(d.uptime);
  }catch(e){$('diff').textContent='Keine Verbindung';$('diff').className='big err';}
}

// Diagramm-Arten: Name, Einheit an der Achse, Werte [innen, außen] je Punkt
const ARTEN={
  tp:{n:'Taupunkt',e:'°',w:q=>[q[4],q[5]]},
  t:{n:'Temperatur',e:'°',w:q=>[q[0],q[2]]},
  rh:{n:'Rel. Feuchte',e:'%',w:q=>[q[1],q[3]]},
  ah:{n:'Abs. Feuchte (g/m³)',e:'',w:q=>[absF(q[0],q[1]),absF(q[2],q[3])]}
};
let art=localStorage.getItem('art'); if(!ARTEN[art])art='tp';
function zeigeArt(){document.querySelectorAll('#seg button').forEach(b=>b.classList.toggle('akt',b.dataset.a===art));}
$('seg').addEventListener('click',e=>{const a=e.target.dataset&&e.target.dataset.a;if(!a)return;art=a;localStorage.setItem('art',a);zeigeArt();zeichne();});
zeigeArt();
let ber=localStorage.getItem('ber')==='7d'?'7d':'24h';
function zeigeBer(){document.querySelectorAll('#ber button').forEach(b=>b.classList.toggle('akt',b.dataset.b===ber));}
$('ber').addEventListener('click',e=>{const b=e.target.dataset&&e.target.dataset.b;if(!b||b===ber)return;ber=b;localStorage.setItem('ber',b);zeigeBer();letzterVerlauf=null;verlauf();});
zeigeBer();
let letzterVerlauf=null;
async function verlauf(){try{const b=ber,v=await (await fetch('/verlauf'+(b==='7d'?'?b=7d':''))).json();if(b!==ber)return;letzterVerlauf=v;zeichne();}catch(e){}}

function zeichne(){
  const v=letzterVerlauf; if(!v)return;
  const c=$('ch'),r=window.devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
  c.width=w*r;c.height=h*r;const g=c.getContext('2d');g.scale(r,r);g.clearRect(0,0,w,h);
  const cs=getComputedStyle(document.documentElement),col=n=>cs.getPropertyValue(n).trim();
  g.font='12px system-ui';
  const A=ARTEN[art],N=v.n,M=v.p.length;
  const p=v.p.map(q=>[...A.w(q),q[6]]);   // [innen, außen, lüfter]
  $('chT').textContent=A.n+' – Verlauf';
  if(M<2){g.fillStyle=col('--mu');g.fillText('Noch zu wenig Daten – alle '+v.iv+' Minuten kommt ein Punkt dazu.',8,h/2);return;}
  // x-Achse: vorhandener Zeitraum, mindestens 12 Punkte, höchstens der ganze Puffer
  const S=Math.min(N-1,Math.max(M-1,12)),min=S*v.iv;
  $('chT').textContent=A.n+' – letzte '+(M>=N?(v.iv>=30?'7 Tage':'24 Stunden'):dauer(min*60));
  const ys=[];p.forEach(q=>{if(q[0]!=null)ys.push(q[0]);if(q[1]!=null)ys.push(q[1]);});
  if(!ys.length)return;
  let lo=Math.floor(Math.min(...ys))-1,hi=Math.ceil(Math.max(...ys))+1;
  if(art==='rh'){lo=Math.max(0,lo);hi=Math.min(100,hi);}
  const L=34,R=6,T=6,B=20,W=w-L-R,H=h-T-B;
  const x=i=>L+W*(S-(M-1)+i)/S, y=val=>T+H*(hi-val)/(hi-lo);
  // Lüfter-Zeiten: Deckkraft = Anteil der Laufzeit, gleiche Werte als ein Rechteck
  g.fillStyle=col('--on');const st=W/S;
  for(let i=0;i<M;i++){
    const f=p[i][2];if(!f)continue;
    let j=i;while(j+1<M&&p[j+1][2]===f)j++;
    const a=Math.round(Math.max(L,x(i)-st/2)),b=Math.round(Math.min(L+W,x(j)+st/2));
    g.globalAlpha=.18*f;g.fillRect(a,T,b-a,H);i=j;
  }
  g.globalAlpha=1;
  // Raster
  g.strokeStyle=col('--bd');g.fillStyle=col('--mu');g.lineWidth=1;
  const schritt=[1,2,5,10,20,25,50].find(s=>s>=(hi-lo)/5)||100;
  for(let t=Math.ceil(lo/schritt)*schritt;t<=hi;t+=schritt){g.beginPath();g.moveTo(L,y(t));g.lineTo(w-R,y(t));g.stroke();g.fillText(t+A.e,2,y(t)+4);}
  const zl=m=>'−'+(m<60?Math.round(m)+' min':m<2880?zf(m/60,m%60?1:0)+' h':zf(m/1440,m%1440?1:0)+' Tage');
  [[0,zl(min)],[.5,zl(min/2)],[1,'jetzt']].forEach(([f,t])=>{const tw=g.measureText(t).width;g.fillText(t,Math.min(Math.max(L+W*f-tw/2,L),w-R-tw),h-4);});
  // Linien
  [[0,'--in'],[1,'--out']].forEach(([k,c2])=>{
    g.strokeStyle=col(c2);g.lineWidth=2;g.beginPath();let auf=false;
    p.forEach((q,i)=>{if(q[k]==null){auf=false;return;}auf?g.lineTo(x(i),y(q[k])):g.moveTo(x(i),y(q[k]));auf=true;});
    g.stroke();
  });
}

$('f').addEventListener('submit',async e=>{
  e.preventDefault();
  try{
    const j=await (await fetch('/einstellungen',{method:'POST',body:new URLSearchParams(new FormData($('f')))})).json();
    $('fm').textContent=j.ok?'Gespeichert.':'Nicht gespeichert: '+j.msg;
    $('fm').className=j.ok?'mu':'mu err';
  }catch(e){$('fm').textContent='Keine Verbindung';$('fm').className='mu err';}
});

window.addEventListener('resize',zeichne);
daten();verlauf();setInterval(daten,5000);setInterval(verlauf,60000);
</script></body></html>)rawliteral";

void zahl(String &s, float v) {
  if (isnan(v)) s += "null"; else s += String(v, 1);
}

void zahl16(String &s, int16_t v) {
  if (v == INT16_MIN) s += "null"; else s += String(v / 10.0f, 1);
}

void handleSeite() {
  server.send_P(200, "text/html; charset=utf-8", SEITE);
}

void handleDaten() {
  unsigned long jetzt = millis();
  String s;
  s.reserve(400);
  s += "{\"ok\":";     s += sensorOk ? "true" : "false";
  s += ",\"fan\":";    s += fanOn ? "true" : "false";
  s += ",\"tIn\":";    zahl(s, mTIn);
  s += ",\"rhIn\":";   zahl(s, mRhIn);
  s += ",\"dIn\":";    zahl(s, mDIn);
  s += ",\"tOut\":";   zahl(s, mTOut);
  s += ",\"rhOut\":";  zahl(s, mRhOut);
  s += ",\"dOut\":";   zahl(s, mDOut);
  s += ",\"seit\":";   s += (jetzt - zustandSeit) / 1000;
  s += ",\"uptime\":"; s += jetzt / 1000;
  s += ",\"rssi\":";   s += WiFi.RSSI();
  s += ",\"diffEin\":"; zahl(s, diffEin);
  s += ",\"diffAus\":"; zahl(s, diffAus);
  s += ",\"tMin\":";    zahl(s, tInnenMin);
  s += ",\"rhMin\":";   zahl(s, rhInnenMin);
  s += ",\"tAMin\":";   zahl(s, tAussenMin);
  s += ",\"maxLauf\":"; s += (int)maxLaufMin;
  s += ",\"pause\":";   s += (int)pauseMin;
  unsigned long pauseRest = 0;
  if (pauseAktiv) {
    unsigned long ende = pauseStart + max(minutenMs(pauseMin), MIN_ZEIT_MS);
    if ((long)(ende - jetzt) > 0) pauseRest = (ende - jetzt) / 1000;
  }
  s += ",\"pauseRest\":"; s += pauseRest;
  s += "}";
  server.send(200, "application/json", s);
}

// /verlauf = 24 h, /verlauf?b=7d = 7 Tage
void handleVerlauf() {
  const Puffer &b = server.arg("b") == "7d" ? lang : kurz;
  String s;
  s.reserve(50 * b.anzahl + 32);
  s += "{\"n\":"; s += b.n;
  s += ",\"iv\":"; s += b.ivS / 60;   // Minuten pro Punkt
  s += ",\"p\":[";
  for (int i = 0; i < b.anzahl; i++) {
    const Punkt &p = b.p[(b.kopf - b.anzahl + i + b.n) % b.n];
    if (i) s += ',';
    s += '[';
    zahl16(s, p.tIn);  s += ','; zahl16(s, p.rhIn);  s += ',';
    zahl16(s, p.tOut); s += ','; zahl16(s, p.rhOut); s += ',';
    zahl16(s, p.dIn);  s += ','; zahl16(s, p.dOut);  s += ',';
    // Lüfter als Anteil 0…1
    if (p.fan == LUEFTER_UNBEKANNT) s += "null";
    else if (p.fan == 0 || p.fan == 100) s += p.fan / 100;
    else s += String(p.fan / 100.0f, 2);
    s += ']';
  }
  s += "]}";
  server.send(200, "application/json", s);
}

void fehlerAntwort(const char *msg) {
  server.send(400, "application/json", String("{\"ok\":false,\"msg\":\"") + msg + "\"}");
}

void handleEinstellungen() {
  if (!server.hasArg("diffEin") || !server.hasArg("diffAus") ||
      !server.hasArg("tMin") || !server.hasArg("rhMin") || !server.hasArg("tAMin") ||
      !server.hasArg("maxLauf") || !server.hasArg("pause")) return fehlerAntwort("Werte fehlen");

  float de = server.arg("diffEin").toFloat();
  float da = server.arg("diffAus").toFloat();
  float tm = server.arg("tMin").toFloat();
  float rm = server.arg("rhMin").toFloat();
  float ta = server.arg("tAMin").toFloat();
  float ml = roundf(server.arg("maxLauf").toFloat());
  float pm = roundf(server.arg("pause").toFloat());

  if (de < 0.5 || de > 20)  return fehlerAntwort("Einschalt-Differenz muss 0,5 bis 20 K sein");
  if (da < 0 || da > de - 0.5) return fehlerAntwort("Ausschalt-Differenz muss mind. 0,5 K unter der Einschalt-Differenz liegen");
  if (tm < -10 || tm > 30)  return fehlerAntwort("Mindesttemperatur muss -10 bis 30 °C sein");
  if (rm < 0 || rm > 95)    return fehlerAntwort("Mindestfeuchte muss 0 bis 95 % sein");
  if (ta < -30 || ta > 15)  return fehlerAntwort("Frostschutz-Temperatur muss -30 bis 15 °C sein");
  if (ml != 0 && (ml < 5 || ml > 240)) return fehlerAntwort("Maximale Laufzeit muss 0 oder 5 bis 240 min sein");
  if (pm < 5 || pm > 240)   return fehlerAntwort("Pause muss 5 bis 240 min sein");

  diffEin = de; diffAus = da; tInnenMin = tm; rhInnenMin = rm;
  tAussenMin = ta; maxLaufMin = ml; pauseMin = pm;
  prefs.putFloat("diffEin", diffEin);
  prefs.putFloat("diffAus", diffAus);
  prefs.putFloat("tMin", tInnenMin);
  prefs.putFloat("rhMin", rhInnenMin);
  prefs.putFloat("tAMin", tAussenMin);
  prefs.putFloat("maxLauf", maxLaufMin);
  prefs.putFloat("pause", pauseMin);
  Serial.printf("Neue Einstellungen: Ein %.1f K, Aus %.1f K, Tmin %.1f C, RHmin %.0f %%, "
                "TAmin %.1f C, max. Lauf %.0f min, Pause %.0f min\n",
                diffEin, diffAus, tInnenMin, rhInnenMin, tAussenMin, maxLaufMin, pauseMin);
  server.send(200, "application/json", "{\"ok\":true}");
}

// ================= Setup / Loop =================

void setup() {
  // Relais als Erstes sicher auf AUS
  pinMode(RELAY_PIN, OUTPUT);
  setRelay(false);

  Serial.begin(115200);
  delay(500);

  prefs.begin("taupunkt", false);
  diffEin    = prefs.getFloat("diffEin", diffEin);
  diffAus    = prefs.getFloat("diffAus", diffAus);
  tInnenMin  = prefs.getFloat("tMin", tInnenMin);
  rhInnenMin = prefs.getFloat("rhMin", rhInnenMin);
  tAussenMin = prefs.getFloat("tAMin", tAussenMin);
  maxLaufMin = prefs.getFloat("maxLauf", maxLaufMin);
  pauseMin   = prefs.getFloat("pause", pauseMin);

  dateisystemOk = LittleFS.begin(true);   // beim ersten Start wird formatiert
  if (dateisystemOk) verlaufLaden();
  else Serial.println("LittleFS nicht verfügbar, Verlauf nur im RAM");

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(false);         // Neuverbinden übernimmt loop()
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  WiFi.setTxPower(WLAN_SENDELEISTUNG);

  server.on("/", HTTP_GET, handleSeite);
  server.on("/favicon.svg", HTTP_GET, []() {
    server.sendHeader("Cache-Control", "max-age=86400");
    server.send_P(200, "image/svg+xml", FAVICON);
  });
  server.on("/daten", HTTP_GET, handleDaten);
  server.on("/verlauf", HTTP_GET, handleVerlauf);
  server.on("/einstellungen", HTTP_POST, handleEinstellungen);
  server.onNotFound([]() { server.send(404, "text/plain", "Nicht gefunden"); });
  server.begin();

  unsigned long jetzt = millis();
  letzterWechsel = jetzt - MIN_ZEIT_MS;  // beim Start sofort schaltbar
  zustandSeit = jetzt;
  letzteMessung = jetzt - MESS_INTERVALL_MS;
  kurz.letzterMs = jetzt - KURZ_IV_S * 1000UL;   // erster 5-min-Punkt sofort
  lang.letzterMs = jetzt;                         // erster 30-min-Mittelwert nach 30 min
}

void loop() {
  server.handleClient();
  if (mdnsGestartet) ArduinoOTA.handle();

  unsigned long jetzt = millis();

  if (WiFi.status() != WL_CONNECTED) {
    // Auto-Reconnect gibt bei schwachem Signal irgendwann auf -> selbst neu verbinden
    if (wlanWegSeit == 0) {
      wlanWegSeit = jetzt | 1;
      Serial.println("WLAN getrennt");
    } else if (jetzt - wlanWegSeit >= WLAN_NEU_MS) {
      // Status: 1 = SSID nicht gefunden, 4 = Verbindung fehlgeschlagen, 6 = getrennt
      Serial.printf("WLAN weiterhin getrennt (Status %d) -> neuer Verbindungsversuch\n", WiFi.status());
      WiFi.disconnect(true);                 // Funk aus: bricht laufenden Versuch sicher ab
      delay(100);
      WiFi.mode(WIFI_STA);
      if (wlanVersuche % 4 == 0) {           // alle 2 min: sichtbare Netze ausgeben
        int n = WiFi.scanNetworks();
        if (n < 0) Serial.printf("  Scan fehlgeschlagen (%d)\n", n);
        bool gefunden = false;
        for (int i = 0; i < n; i++) {
          bool unser = WiFi.SSID(i) == WLAN_SSID;
          gefunden |= unser;
          Serial.printf("  %s%-24s %4d dBm  Kanal %d\n", unser ? "* " : "  ",
                        WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
        }
        if (!gefunden) Serial.println("  -> eigenes Netz nicht sichtbar");
        WiFi.scanDelete();
      }
      WiFi.begin(WLAN_SSID, WLAN_PASS);
      WiFi.setTxPower(WLAN_SENDELEISTUNG);
      wlanWegSeit = jetzt | 1;
      wlanVersuche++;
      if (wlanVersuche >= 120) {
        // nach ca. 1 h ohne WLAN: Neustart (Lüfter vorher sicher aus)
        Serial.println("WLAN-Neustart des ESP32");
        setRelay(false);
        delay(100);
        ESP.restart();
      }
    }
  } else {
    if (wlanWegSeit != 0) {
      Serial.printf("WLAN wieder verbunden: http://%s (RSSI %d dBm)\n",
                    WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
    wlanWegSeit = 0;
    wlanVersuche = 0;
    if (!mdnsGestartet) {
      mdnsGestartet = true;
      if (MDNS.begin(HOSTNAME)) {
        MDNS.addService("http", "tcp", 80);
        MDNS.enableArduino(3232, true);      // Board in der Arduino-IDE als Netzwerk-Port sichtbar
      }
      ArduinoOTA.setHostname(HOSTNAME);
      ArduinoOTA.setPassword(OTA_PASS);
      ArduinoOTA.setMdnsEnabled(false);      // mDNS läuft bereits oben
      ArduinoOTA.onStart([]() { setRelay(false); });  // Lüfter beim Update aus
      ArduinoOTA.begin();
      configTime(0, 0, "pool.ntp.org", "time.nist.gov");   // UTC, nur für den Verlauf
      Serial.printf("WLAN verbunden: http://%s  bzw.  http://%s.local\n",
                    WiFi.localIP().toString().c_str(), HOSTNAME);
    }
  }

  if (jetzt - letzteMessung >= MESS_INTERVALL_MS) {
    letzteMessung = jetzt;
    messen(jetzt);
  }
}
