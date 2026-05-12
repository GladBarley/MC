

# Mikrocontroller-Praktikum: AVR-C Line Follower

Dieses Projekt entstand im Rahmen des Mikrocontroller-Praktikums als Abschlusspräsentation. Ziel ist die Entwicklung einer robusten Steuerung für einen Line Follower (Linienverfolger) auf Basis der AVR-Architektur in reinem **AVR-C**.

## Projektübersicht

Der Roboter nutzt zwei Infrarot-Sensoren, um eine schwarze Linie auf hellem Untergrund zu detektieren. Je nach Hardware-Konfiguration (Drahtbrücke an Pin `PD4`) wechselt das System zwischen zwei verschiedenen Steuerungsalgorithmen.

### Features

* **Duale Betriebsmodi:** Wahl zwischen einfacher Logik und präziser Regelungstechnik.
* **EEPROM-Integration:** Dauerhafte Speicherung von Kalibrierungsdaten.
* **Echtzeit-Anpassung:** Geschwindigkeitssteuerung via Potentiometer.
* **Hardware-nahe Programmierung:** Direkte Registerzugriffe für maximale Performance.

---

## Betriebsmodi

Das Programm prüft beim Systemstart den Zustand von Pin `PD4`.

### 1. Bang-Bang-Steuerung (Standard)

*Aktiviert, wenn keine Drahtbrücke zwischen `PD4` und `GND` besteht.*

Dieser Modus nutzt eine diskrete Entscheidungslogik:

* **Beide Sensoren Schwarz:** Geradeausfahrt.
* **Ein Sensor Weiß:** Kurskorrektur in die entsprechende Richtung.
* **Beide Sensoren Weiß:** Rückwärtsfahrt (Linie verloren).

**Zusatzfunktionen:**

* **Kalibrierung:** Der Schwellenwert für "Weiß" wird über den Taster **SW2** eingelernt und im **EEPROM** hinterlegt.
* **Auto-Load:** Beim Bootvorgang wird der letzte Schwellenwert automatisch aus dem EEPROM geladen.
* **Speed-Control:** Die Basisgeschwindigkeit wird über das Potentiometer **POT1** geregelt.

### 2. P-Regelung (Proportionalregler)

*Aktiviert, wenn `PD4` mit `GND` verbunden ist.*

Für eine sanftere und stabilere Spurhaltung wird die Differenz der Sensorwerte kontinuierlich ausgewertet. Die Motorgeschwindigkeit wird proportional zur Abweichung angepasst, um Schlangenlinien (Oszillationen) zu minimieren.
