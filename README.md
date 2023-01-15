## Gruppe 04
Gruppenmitglieder: Lukas Baldner, Jonas Strittmatter, Julian Shen

## Aufbau des Roboters


## Protokoll

### 24.10.22
- Sensoren ausprobiert
- Idenn zur Roboterarchitektur gesammelt
- Erster Prototyp für den Roboter gebaut
- Vertrautmachen mit Sensor-Komponenten und Pybricks-API, Einrichten von VS-Code

### 07.11.22
- basic Skelett (Roboter-Klasse, erstes Testen des Line-Following-Algorithmus)
- Roboter umbauen, Ketten statt Räder
- Übungsblatt nacharbeiten
- Roboter fertig gebaut
- Programm für das Linien folgen / Rampe hochfahren konfiguieren

### 07.11.22 - 14.11.22
- Menü zur Auswahl der Routine für den Roboter
- Zustandsdiagramm für das Linienfolgen erstellt

### 14.11.22
- Vorlesung über Regelung angehört
- Menü ausprobiert
- Proportionen des "Auswahlrechtecks" angepasst
- Bug, dass durch zu langes Drücken der Auswahlknöpfe die Auswahl mehr als ein Feld nach oben oder unten geht behoben
- Durch Drücken des Enter-Knopfes im Menü führt der Roboter nun auch die ausgewählte Funktion aus
- Implementierung der Routine fürs Linienfolgen anhand des Zustandsdiagramms, welches durch das letzte Übungsblatt erarbeitet wurde
- Anpassung der Parameter für DriveBase, das Fehler beim Winkelmessen beobachtet wurde
- Konfigurieren weiterer Parameter wie Drehwinkel und Fahrzeit bis er das Linienfolgen erfolgreich schafft
- Roboter erkennt blaue Linien und wechselt automatisch in die nächste Routine, und hält nach der letzten Rotuine an

### 21.11.22
- Aufteilung des Programms in mehrere Dateien
- Ändern des vorher noch blockierenden Drehens in ein nicht-blockierendes Drehen
- Einführung von Flags, die Zustandswechsel während dem Fahren/Drehen ermöglichen
- Zustandsautomaten neu zeichnen, da Programmstruktur verändert wurde
- Wechsel zu State-Method-Entwurfsmuster, um den Zustandsautomaten im Code umzusetzen und leichtere Erweiterbarkeit zu erreichen

### 21.11.22 - 28.11.22
- Alle Zustände für den Wechsel zum State-Method-Entwurfsmuster ergänzt und implementiert

### 28.11.22
- Bugs in den Zuständen behoben
- Lichtsensor umgedreht, damit das Erkennen der blauen Linie funktioniert
- Zustände zum Umfahren des Hindernisses implemen tiert
- Code sauberer gemacht

### 05.12.2022
- Roboter erkennt in der Lücken-Routine nach dem Zurückdrehen die weiße Linie
- Anpassen der Parameter, wie DRIVE_SPEED, TURN_RATE..., sodass das Linienfolgen deutlich flüssiger funktioniert
- Nutzen des Touchsensors bei dem Rückwärtsfahren beim Obstacle
- Code für Farbfeldsuche angefangen: Zustände für das Fahren erstellt
- TODO: Rückwärtsfahren zum Ausrichten, Farben erkennen, Farbsensor korrekt drehen

### 12.12.2022
- Motor für Farbsensor umgebaut, um das Spiel des Arms zu reduzieren
- Farbsensorarm schwenkt jetzt
- Roboter erkennt rot und weiß
- Rückwärtsfahren zum Ausrichten
- Da Farbsensor eher links am Roboter, Routine beim Absuchen des Feldes angepasst
- Geschwindigkeit des Roboters im Feld reduziert, um mit dem Arm den Boden genau genug absuchen zu können

### 12.12.22 - 19.12.22
- Algorithmus zum Paket in die Ecke schieben überlegt (ohne zur Brücke fahren)

### 19.12.2022
- Suchfeld fertig implementiert
- Roboter fährt bei Delivery zum Suchspot
- Suche nach richtiger Drehgeschwindigkeit und Implementierung zum Suchen der Ecke der Box
- Roboter schiebt das Paket fast in die Ecke

### 09.01.2023
- Refactoring: bei Zuständen, die mit "!" beginnen wird nun automatisch der aktuelle
  Zustand des Roboters gespeichert. Das spart viel duplizierten Code :)
- Testen des Ultraschall-Sensors (Plotten der Sensordaten), um Realisierbarkeit unserer
  Strategie für das Paket-Level zu prüfen
- Umbau des Roboters; Ziel: Ultraschall-Sensor soll beweglich sein und Farb- und
  Ultraschall-Sensor sollen beide gleichzeitig nach vorne ausgerichtet sein

### 09.01.23 - 16.01.23
- Überlegungen aufgeschrieben für Brücke überqueren

