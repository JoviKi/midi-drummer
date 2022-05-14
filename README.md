# Code für Semesterarbeit mit dem Thema _**Entwicklung einer adaptiven Steuerung für einen schlagzeugspielenden Roboterarm**_ 

## ROS-Node [`pose_estimation`](./src/drummer/scripts/pose_estimation)

*  Zur Erkennung der ArUco-Codes, die auf den Instrumenten angebracht sind und Übermittlung der Posen-Informationen

## ROS-Node [`midi_translation`](./src/drummer/scripts/midi_translation)
* Extraktion der Spielinformationen (Schlagzeitpunkt und -instrument) aus einem beliebigen MIDI-File
* Kombination der Posen-Informationen aus pose_estimation und Spielinformationen
* Transformation der Posen-Koordinaten in das Roboterkoordinatensystem
* Senden der Posen-Koordinaten an den Roboter
* Senden der Schlagzeitpunkte an Greifer
## ROS-Node [`servo_handler`](./src/servo_handler/scripts/servo_handler.py)
* Empfangen der Schlagzeitpunkte und Umwandlung der Informationen
* Senden der entsprechenden Signale zur Initialisierung der Schläge an den Greifer

## Launch-File [`drummer.launch`](./src/drummer/launch/drummer.launch)
* Zum Starten von pose_estimation, midi_translation und der Koordinatentransformation von Kamera zum Flansch des Roboters 

## Python-Skript [`camera_test.py`](./src/drummer/scripts/camera_test.py)
* Zum Testen der angeschlossenen Kamera

## Python-Skript [`camera_calibration.py`](./camera_calibration.py)
* Zur vorherigen Kalibrierung der verwendeten Kamera

# Anleitung bei erster Inbetriebnahme für Bewegung des UR10e
1. Vorbereitung des PCs und des UR10e mit Hilfe folgender Anleitung: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
2. Anschließen der Kamera an den PC
3. Kalibrierung der Kamera

    3.1. Aufnahme von mind. 10 Fotos eines Schachbrettes aus verschiedenen Distanzen und Winkeln

    3.2. Anpassen der Parameter unter "Chessboard dimensions" und des Pfades, unter dem die aufgenommenen Fotos gefunden werden können

    3.3. Ausführen mittels `python3 camera_calibration.py`

4. Öffnen eines Terminals, Starten eines Roscores mittels `roscore`
5. Öffnen eines Terminals, Starten des Teibers für den UR10e mit folgenden Befehlen:
    ```bash
    cd catkin_ws
    source devel setup.bash
    roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.0.101 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
    ```
6. Starten des Programmes _External Control_ auf dem UR10e
7. Öffnen eines Terminals, Starten des Launch-Files:
    ```bash
    cd midi_drummer
    source devel setup.bash
    roslaunch drummer drummer.launch
    ```

# Häufige Fehlermeldungen

### ArUco-Codes können nicht erkannt werden
* Kamera aus- und wieder einstecken

### Es werden nicht alle ArUco-Codes erkannt
* Roboter nahe zur Ausgangsposition bringen und Programm erneut starten
  (Programm hat nicht lang genug gewartet, um alle Codes erkennen zu können)

### Nach dem Start des Launchers passiert nichts
* Launcher, Roboter-Treiber und "External Control" auf dem Roboter beenden und neu starten:
  erst Treiber, dann External Control und dann den Launcher
* Launcher erst starten, wenn im Terminal des Treibers "Ready to receive commands" steht


___

# Arduino Code zur Steuerung der Bewegung des Greifers

## Vorbereitung
1. Anschließen des Arduinos per USB an den PC
2. Anschließen der Energieversorgung an den Arduino

Informationen zur Handhabung des Greifers sind zu finden unter https://gitlab.lrz.de/ge72wan/code_semesterarbeit.git