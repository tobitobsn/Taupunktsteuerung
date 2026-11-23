# Taupunkt-Lüftersteuerung

Diese Steuerung schaltet einen Lüfter nur dann ein, wenn das Lüften einen Raum (z. B. Keller) wirklich trocknet. Dafür vergleicht sie den Taupunkt innen und außen. Sie läuft auf einem ESP32-C3 mit zwei SHT31-Sonden und einem Relais. Eine Webseite im WLAN zeigt die aktuellen Werte und den 24-h-Verlauf. Über die Webseite lassen sich auch die Schwellen einstellen.

## Funktionsweise

Alle 5 s misst der ESP32 Temperatur und relative Feuchte innen und außen. Die Messwerte werden mit einem gleitenden Mittelwert (Zeitkonstante 2 min, `GLAETTUNG_S`) geglättet, damit Wind, Regen oder Sonne am Außensensor die Regelung nicht unruhig machen. Daraus berechnet er mit der Magnus-Formel den Taupunkt (Koeffizienten a = 17,62, b = 243,12 °C):

```
γ  = ln(RH/100) + a·T/(b+T)
Td = b·γ / (a − γ)
```

Ist die Luft außen trockener als innen, also der Taupunkt außen niedriger, dann trocknet Lüften den Raum.

| | Bedingung |
|---|---|
| **Ein** | Td innen − Td außen > `diffEin` **und** T innen > `tInnenMin` **und** rF innen > `rhInnenMin` **und** T außen ≥ `tAussenMin` **und** keine Pause |
| **Aus** | Differenz < `diffAus` **oder** T innen < `tInnenMin` **oder** rF innen < `rhInnenMin` |

- Zwischen zwei Schaltvorgängen liegen mindestens 5 min, sowohl als Laufzeit als auch als Pause.
- Fällt ein Sensor aus oder liefert unplausible Werte, geht der Lüfter **sofort aus**, auch vor Ablauf der Mindestzeit.
- **Frostschutz:** Fällt die Außentemperatur unter `tAussenMin`, geht der Lüfter ebenfalls sofort aus. So bläst die Steuerung im Winter keine eiskalte Luft auf Leitungen.
- **Maximale Laufzeit:** Ist `maxLaufMin` größer als 0, schaltet der Lüfter nach dieser Zeit für `pauseMin` ab (mindestens aber 5 min). Das kühlt den Raum weniger aus und schont den Lüfter.
- Die Regelung läuft auch ohne WLAN-Verbindung weiter.

Standardwerte, die sich über die Webseite ändern lassen und im Flash gespeichert werden:

| Parameter | Standard | Bedeutung |
|---|---|---|
| `diffEin` | 5,0 K | Taupunkt-Differenz zum Einschalten |
| `diffAus` | 2,0 K | Taupunkt-Differenz zum Ausschalten (Hysterese) |
| `tInnenMin` | 10 °C | darunter nicht lüften (Auskühlen) |
| `rhInnenMin` | 50 % | darunter gibt es nichts zu trocknen |
| `tAussenMin` | −5 °C | Frostschutz: darunter nicht lüften |
| `maxLaufMin` | 0 min | maximale Laufzeit am Stück (0 = unbegrenzt) |
| `pauseMin` | 10 min | Pause nach Erreichen der maximalen Laufzeit |

## Hardware

- ESP32-C3 (SuperMini oder DevKitM)
- 2× SHT31-Sonde (z. B. FS400-SHT3X), beide auf Adresse 0x44
- Relaismodul für den 230-V-Lüfter

Weil beide Sonden dieselbe I²C-Adresse haben, hängt jede an eigenen Pins. Vor jeder Messung schaltet der Sketch den Bus auf die jeweilige Sonde um. Der Takt ist wegen der langen Kabel (ca. 1,5 m) auf 50 kHz reduziert.

| Signal | GPIO |
|---|---|
| Sonde innen SDA / SCL | 9 / 8 |
| Sonde außen SDA / SCL | 7 / 6 |
| Relais | 4 |

Ist dein Relaismodul *active LOW*, setze im Sketch `RELAY_ACTIVE_LOW = true`. Weichen die Sensoren voneinander ab, kannst du sie über die Konstanten `T_OFFSET_*` und `RH_OFFSET_*` korrigieren.

Der Innensensor sollte nicht direkt im Luftstrom des Lüfters und nicht direkt an einer feuchten Wand hängen, sonst misst er nicht die Raumluft. Am Einbauort sollte das WLAN-Signal besser als etwa −80 dBm sein. Bei schwächerem Empfang helfen ein Repeater oder ein ESP32-Board mit Anschluss für eine externe Antenne (u.FL).

> ⚠️ Am Relais liegt Netzspannung (230 V) an. Installation nur durch eine Elektrofachkraft.

## Einrichtung

1. Kopiere `wlan_daten.example.h` nach `wlan_daten.h` und trage dort WLAN-Name, WLAN-Passwort (nur 2,4 GHz) und ein OTA-Passwort ein. `wlan_daten.h` steht in `.gitignore` und wird nicht eingecheckt.
2. Installiere in der Arduino-IDE den Boardkern **esp32** von Espressif (getestet mit 3.3.12). Weitere Bibliotheken braucht der Sketch nicht.
3. Wähle als Board **ESP32C3 Dev Module** und stelle **USB CDC On Boot: Enabled** ein.
4. Lade den Sketch beim ersten Mal per USB hoch.

Mit `arduino-cli` geht das so (der Sketch-Ordner muss `taupunkt_luefter` heißen):

```sh
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc --output-dir build taupunkt_luefter
arduino-cli upload  --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc -p /dev/cu.usbmodem101 --input-dir build
```

## WLAN-Verbindung

Reißt die WLAN-Verbindung ab, versucht der ESP32 alle 30 s, sich neu anzumelden. Bekommt er eine Stunde lang keine Verbindung, startet er neu und schaltet vorher den Lüfter aus. Die Regelung läuft in der Zwischenzeit normal weiter. Nach einem Neustart ist der Verlauf leer.

Der serielle Monitor (115200 Baud) zeigt, was mit dem WLAN los ist:

```
WLAN weiterhin getrennt (Status 5) -> neuer Verbindungsversuch
  * MeinWLAN                  -92 dBm  Kanal 3
```

- **Status:** 1 = Netz nicht gefunden, 4 = Anmeldung fehlgeschlagen (z. B. falsches Passwort), 5 = Verbindung verloren, 6 = getrennt.
- **Netzliste:** Alle 2 min listet der ESP32 die Netze auf, die er sieht. Das eigene Netz ist mit `*` markiert. Liegt es bei etwa −90 dBm oder schwächer, ist der Empfang zu schlecht für eine stabile Verbindung.

Nach einem Neustart kann der Router dem ESP32 eine neue IP-Adresse geben, und `taupunkt.local` funktioniert nicht in jedem Netz zuverlässig. Damit die Adresse gleich bleibt, richte im Router eine DHCP-Reservierung für den ESP32 ein. Die aktuelle Adresse steht nach dem Start im seriellen Monitor (`WLAN verbunden: http://…`).

## Update über WLAN (OTA)

Nach dem ersten USB-Upload kannst du jeden weiteren Sketch über WLAN hochladen. In der Arduino-IDE erscheint das Board als Netzwerk-Port `taupunkt`. Beim Upload fragt die IDE nach `OTA_PASS`. Während des Updates schaltet der ESP32 den Lüfter aus.

```sh
arduino-cli upload --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc \
  -p <IP-Adresse> --protocol network \
  --upload-field password=<OTA_PASS> --input-dir build
```

## Webseite

Die Webseite ist unter **http://taupunkt.local** erreichbar, alternativ unter der IP-Adresse, die der serielle Monitor ausgibt. Sie zeigt:

- Temperatur, Feuchte und Taupunkt innen und außen
- den Lüfterstatus mit Laufzeit seit dem letzten Wechsel und, wenn er aus ist, den Grund dafür
- den Verlauf der letzten 24 h (ein Punkt alle 5 min; liegt im RAM und ist nach einem Neustart leer). Solange noch keine 24 h aufgezeichnet sind, zeigt das Diagramm nur den vorhandenen Zeitraum (mindestens 1 h).
- ein Formular für die Schwellen

| Pfad | Methode | Inhalt |
|---|---|---|
| `/` | GET | Webseite |
| `/daten` | GET | aktuelle Werte als JSON |
| `/verlauf` | GET | 24-h-Verlauf als JSON |
| `/einstellungen` | POST | Schwellen setzen (`diffEin`, `diffAus`, `tMin`, `rhMin`, `tAMin`, `maxLauf`, `pause`) |
| `/favicon.svg` | GET | Icon (Lüfterblatt) |
