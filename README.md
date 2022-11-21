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

nested loop für lücke umschreiben, Funktion zum passiven drehen, hindernis