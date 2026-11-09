/*
  Taupunkt-Lüftersteuerung mit Webseite
  Board:    ESP32-C3 (SuperMini / DevKitM), "USB CDC On Boot: Enabled"
  Sensoren: 2x SHT31-Sonde (FS400-SHT3X), beide fest auf Adresse 0x44
            -> jede Sonde an eigenen Pins, der I2C-Bus wird vor jeder
               Messung auf die Pins der jeweiligen Sonde umgeschaltet
  Ausgang:  Relaismodul für 230-V-Lüfter
  Upload:   per USB oder über WLAN (OTA, Passwort OTA_PASS in wlan_daten.h)
  Web:      http://taupunkt.local  (oder IP aus dem seriellen Monitor)
            Aktuelle Werte, 24-h-Verlauf, Schwellen einstellbar

  WLAN-Zugangsdaten in wlan_daten.h eintragen.
  Keine externe Bibliothek nötig.

  Logik:
    Ein   : Td_innen - Td_außen > diffEin  UND Tinnen > tInnenMin UND RHinnen > rhInnenMin
    Aus   : Differenz < diffAus ODER Tinnen < tInnenMin ODER RHinnen < rhInnenMin
    Mindestlaufzeit und Mindestpause je MIN_ZEIT_MS
    Sensorfehler / unplausible Werte -> Lüfter sofort AUS (hat Vorrang vor Mindestzeiten)
    Die Regelung läuft unabhängig davon, ob WLAN verbunden ist.
*/

#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
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

const unsigned long MESS_INTERVALL_MS = 5000;                 // alle 5 s messen
const unsigned long MIN_ZEIT_MS       = 5UL * 60UL * 1000UL;  // 5 min Mindestlauf/-pause

// Korrekturwerte (Offset) falls Sensoren abweichen
const float T_OFFSET_INNEN = 0.0, RH_OFFSET_INNEN = 0.0;
const float T_OFFSET_AUSSEN = 0.0, RH_OFFSET_AUSSEN = 0.0;

// Verlauf: alle 5 min ein Punkt, 288 Punkte = 24 h
const int VERLAUF_N = 288;
const unsigned long VERLAUF_INTERVALL_MS = 5UL * 60UL * 1000UL;
// -----------------------------------------------

WebServer server(80);
Preferences prefs;

bool fanOn = false;
unsigned long letzterWechsel = 0;   // für Mindestzeiten
unsigned long zustandSeit = 0;      // für Anzeige "seit ..."
unsigned long letzteMessung = 0;
bool mdnsGestartet = false;

// aktuelle Messwerte
float mTIn = NAN, mRhIn = NAN, mTOut = NAN, mRhOut = NAN, mDIn = NAN, mDOut = NAN;
bool sensorOk = false;

// Verlauf (Werte x10 als int16, INT16_MIN = kein Wert)
struct Punkt { int16_t tIn, rhIn, tOut, rhOut, dIn, dOut; uint8_t fan; };
Punkt verlauf[VERLAUF_N];
int vKopf = 0, vAnzahl = 0;
unsigned long letzterVerlauf = 0;
bool verlaufGestartet = false;

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

int16_t kodiere(float v) { return isnan(v) ? INT16_MIN : (int16_t)lroundf(v * 10.0f); }

void verlaufSpeichern() {
  Punkt &p = verlauf[vKopf];
  p.tIn = kodiere(mTIn);   p.rhIn = kodiere(mRhIn);
  p.tOut = kodiere(mTOut); p.rhOut = kodiere(mRhOut);
  p.dIn = kodiere(mDIn);   p.dOut = kodiere(mDOut);
  p.fan = fanOn ? 1 : 0;
  vKopf = (vKopf + 1) % VERLAUF_N;
  if (vAnzahl < VERLAUF_N) vAnzahl++;
}

void messen(unsigned long jetzt) {
  float tIn, rhIn, tOut, rhOut;
  bool okIn  = leseSht(SDA_INNEN,  SCL_INNEN,  tIn,  rhIn);
  bool okOut = leseSht(SDA_AUSSEN, SCL_AUSSEN, tOut, rhOut);

  tIn  += T_OFFSET_INNEN;   rhIn  = constrain(rhIn  + RH_OFFSET_INNEN,  0.0, 100.0);
  tOut += T_OFFSET_AUSSEN;  rhOut = constrain(rhOut + RH_OFFSET_AUSSEN, 0.0, 100.0);

  sensorOk = okIn && okOut && plausibel(tIn, rhIn) && plausibel(tOut, rhOut);

  if (!sensorOk) {
    // ---- Sensorfehler: Lüfter sofort aus ----
    mTIn = mRhIn = mTOut = mRhOut = mDIn = mDOut = NAN;
    if (okIn)  { mTIn = tIn;   mRhIn = rhIn; }
    if (okOut) { mTOut = tOut; mRhOut = rhOut; }
    Serial.printf("Sensorfehler (innen %s, aussen %s) -> Luefter AUS\n",
                  okIn ? "OK" : "FEHLER", okOut ? "OK" : "FEHLER");
    schalte(false, jetzt);
  } else {
    mTIn = tIn; mRhIn = rhIn; mTOut = tOut; mRhOut = rhOut;
    mDIn  = taupunkt(tIn, rhIn);
    mDOut = taupunkt(tOut, rhOut);
    float diff = mDIn - mDOut;

    bool zeitOk = (jetzt - letzterWechsel) >= MIN_ZEIT_MS;

    if (!fanOn && zeitOk && diff > diffEin && tIn > tInnenMin && rhIn > rhInnenMin) {
      schalte(true, jetzt);
    } else if (fanOn && zeitOk && ((diff < diffAus) || (tIn < tInnenMin) || (rhIn < rhInnenMin))) {
      schalte(false, jetzt);
    } else {
      setRelay(fanOn);
    }

    Serial.printf("In: %.1f C %.1f %% Td %.1f | Out: %.1f C %.1f %% Td %.1f | Diff %.1f K | Luefter %s\n",
                  tIn, rhIn, mDIn, tOut, rhOut, mDOut, diff, fanOn ? "EIN" : "AUS");
  }

  if (!verlaufGestartet || jetzt - letzterVerlauf >= VERLAUF_INTERVALL_MS) {
    verlaufSpeichern();
    letzterVerlauf = jetzt;
    verlaufGestartet = true;
  }
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
    <div class="row"><span>Luftfeuchte</span><span id="rhIn">–</span></div></div>
  <div class="card"><div class="lbl" style="color:var(--out)">Außen</div>
    <div class="big" id="dOut">–</div><div class="mu">Taupunkt</div>
    <div class="row"><span>Temperatur</span><span id="tOut">–</span></div>
    <div class="row"><span>Luftfeuchte</span><span id="rhOut">–</span></div></div>
</div>

<div class="card"><div class="lbl">Taupunkt – letzte 24 Stunden</div>
  <canvas id="ch"></canvas>
  <div class="mu leg"><span style="color:var(--in)">━ innen</span><span style="color:var(--out)">━ außen</span><span style="color:var(--on)">▮ Lüfter an</span></div>
</div>

<div class="card"><div class="lbl">Einstellungen</div>
  <form id="f">
    <label for="diffEin">Einschalten ab Taupunkt-Differenz (K)</label><input id="diffEin" name="diffEin" type="number" step="0.5" min="0.5" max="20">
    <label for="diffAus">Ausschalten unter Differenz (K)</label><input id="diffAus" name="diffAus" type="number" step="0.5" min="0" max="19.5">
    <label for="tMin">Innen-Temperatur mindestens (°C)</label><input id="tMin" name="tMin" type="number" step="0.5" min="-10" max="30">
    <label for="rhMin">Innen-Feuchte mindestens (%)</label><input id="rhMin" name="rhMin" type="number" step="1" min="0" max="95">
    <button>Speichern</button>
  </form>
  <div class="mu" id="fm" style="margin-top:8px"></div>
</div>
<div class="mu" id="info"></div>
</main>
<script>
const $=id=>document.getElementById(id);
const f1=v=>v==null?'–':v.toFixed(1);
function dauer(s){if(s<3600)return Math.floor(s/60)+' min';if(s<86400)return (s/3600).toFixed(1).replace('.',',')+' h';return (s/86400).toFixed(1).replace('.',',')+' Tage';}
let formGeladen=false;

async function daten(){
  try{
    const d=await (await fetch('/daten')).json();
    $('dIn').textContent=f1(d.dIn)+' °C';  $('dOut').textContent=f1(d.dOut)+' °C';
    $('tIn').textContent=f1(d.tIn)+' °C';  $('tOut').textContent=f1(d.tOut)+' °C';
    $('rhIn').textContent=f1(d.rhIn)+' %'; $('rhOut').textContent=f1(d.rhOut)+' %';
    const b=$('fan'); b.textContent=d.fan?'Lüfter EIN':'Lüfter AUS';
    b.style.background=d.fan?'var(--on)':'var(--off)';
    const df=$('diff');
    if(d.ok){df.textContent='Differenz '+f1(d.dIn-d.dOut)+' K';df.className='big';}
    else{df.textContent='Sensorfehler';df.className='big err';}
    df.style.fontSize='1.4rem';
    $('seit').textContent=(d.fan?'an':'aus')+' seit '+dauer(d.seit);
    if(!formGeladen){$('diffEin').value=d.diffEin;$('diffAus').value=d.diffAus;$('tMin').value=d.tMin;$('rhMin').value=d.rhMin;formGeladen=true;}
    $('info').textContent='WLAN '+d.rssi+' dBm · in Betrieb seit '+dauer(d.uptime);
  }catch(e){$('diff').textContent='Keine Verbindung';$('diff').className='big err';}
}

let letzterVerlauf=null;
async function verlauf(){try{letzterVerlauf=await (await fetch('/verlauf')).json();zeichne();}catch(e){}}

function zeichne(){
  const v=letzterVerlauf; if(!v)return;
  const c=$('ch'),r=window.devicePixelRatio||1,w=c.clientWidth,h=c.clientHeight;
  c.width=w*r;c.height=h*r;const g=c.getContext('2d');g.scale(r,r);g.clearRect(0,0,w,h);
  const cs=getComputedStyle(document.documentElement),col=n=>cs.getPropertyValue(n).trim();
  g.font='12px system-ui';
  const p=v.p,N=v.n;
  if(p.length<2){g.fillStyle=col('--mu');g.fillText('Noch zu wenig Daten – alle 5 Minuten kommt ein Punkt dazu.',8,h/2);return;}
  const ys=[];p.forEach(q=>{if(q[4]!=null)ys.push(q[4]);if(q[5]!=null)ys.push(q[5]);});
  if(!ys.length)return;
  let lo=Math.floor(Math.min(...ys))-1,hi=Math.ceil(Math.max(...ys))+1;
  const L=34,R=6,T=6,B=20,W=w-L-R,H=h-T-B;
  const x=i=>L+W*(N-p.length+i)/(N-1), y=val=>T+H*(hi-val)/(hi-lo);
  // Lüfter-Zeiten
  g.fillStyle=col('--on');g.globalAlpha=.18;const st=W/(N-1);
  p.forEach((q,i)=>{if(q[6])g.fillRect(x(i)-st/2,T,st+.5,H);});
  g.globalAlpha=1;
  // Raster
  g.strokeStyle=col('--bd');g.fillStyle=col('--mu');g.lineWidth=1;
  const schritt=Math.max(1,Math.ceil((hi-lo)/5));
  for(let t=Math.ceil(lo/schritt)*schritt;t<=hi;t+=schritt){g.beginPath();g.moveTo(L,y(t));g.lineTo(w-R,y(t));g.stroke();g.fillText(t+'°',2,y(t)+4);}
  [[0,'−24 h'],[.5,'−12 h'],[1,'jetzt']].forEach(([f,t])=>{const tw=g.measureText(t).width;g.fillText(t,Math.min(Math.max(L+W*f-tw/2,L),w-R-tw),h-4);});
  // Linien
  [[4,'--in'],[5,'--out']].forEach(([k,c2])=>{
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
  s.reserve(320);
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
  s += "}";
  server.send(200, "application/json", s);
}

void handleVerlauf() {
  String s;
  s.reserve(48 * vAnzahl + 32);
  s += "{\"n\":"; s += VERLAUF_N; s += ",\"p\":[";
  for (int i = 0; i < vAnzahl; i++) {
    const Punkt &p = verlauf[(vKopf - vAnzahl + i + VERLAUF_N) % VERLAUF_N];
    if (i) s += ',';
    s += '[';
    zahl16(s, p.tIn);  s += ','; zahl16(s, p.rhIn);  s += ',';
    zahl16(s, p.tOut); s += ','; zahl16(s, p.rhOut); s += ',';
    zahl16(s, p.dIn);  s += ','; zahl16(s, p.dOut);  s += ',';
    s += p.fan;
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
      !server.hasArg("tMin") || !server.hasArg("rhMin")) return fehlerAntwort("Werte fehlen");

  float de = server.arg("diffEin").toFloat();
  float da = server.arg("diffAus").toFloat();
  float tm = server.arg("tMin").toFloat();
  float rm = server.arg("rhMin").toFloat();

  if (de < 0.5 || de > 20)  return fehlerAntwort("Einschalt-Differenz muss 0,5 bis 20 K sein");
  if (da < 0 || da > de - 0.5) return fehlerAntwort("Ausschalt-Differenz muss mind. 0,5 K unter der Einschalt-Differenz liegen");
  if (tm < -10 || tm > 30)  return fehlerAntwort("Mindesttemperatur muss -10 bis 30 °C sein");
  if (rm < 0 || rm > 95)    return fehlerAntwort("Mindestfeuchte muss 0 bis 95 % sein");

  diffEin = de; diffAus = da; tInnenMin = tm; rhInnenMin = rm;
  prefs.putFloat("diffEin", diffEin);
  prefs.putFloat("diffAus", diffAus);
  prefs.putFloat("tMin", tInnenMin);
  prefs.putFloat("rhMin", rhInnenMin);
  Serial.printf("Neue Einstellungen: Ein %.1f K, Aus %.1f K, Tmin %.1f C, RHmin %.0f %%\n",
                diffEin, diffAus, tInnenMin, rhInnenMin);
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

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // stabiler bei der SuperMini-Antenne

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
}

void loop() {
  server.handleClient();
  if (mdnsGestartet) ArduinoOTA.handle();

  if (WiFi.status() == WL_CONNECTED) {
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
      Serial.printf("WLAN verbunden: http://%s  bzw.  http://%s.local\n",
                    WiFi.localIP().toString().c_str(), HOSTNAME);
    }
  }

  unsigned long jetzt = millis();
  if (jetzt - letzteMessung >= MESS_INTERVALL_MS) {
    letzteMessung = jetzt;
    messen(jetzt);
  }
}
