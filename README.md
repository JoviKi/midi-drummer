##### Code für Semesterarbeit mit dem Thema "Entwicklung einer adaptiven Steuerung für einen schlagzeugspielenden Roboterarm" #####

# Inhalt
1. ROS-Node "pose_estimation"
* Zur Erkennung der ArUco-Codes, die auf den Instrumenten angebracht sind und Übermittlung der Posen-Informationen
2. ROS-Node "midi_translation"
* Extraktion der Spielinformationen (Schlagzeitpunkt und -instrument) aus einem beliebigen MIDI-File
* Kombination der Posen-Informationen aus pose_estimation und Spielinformationen
* Transformation der Posen-Koordinaten in das Roboterkoordinatensystem
* Senden der Posen-Koordinaten an den Roboter
* Senden der Schlagzeitpunkte an Greifer
3. ROS-Node "servo_control"
* Empfangen der Schlagzeitpunkte und Umwandlung der Informationen
* Senden der entsprechenden Signale zur Initialisierung der Schläge an den Greifer
4. Launch-File "drummer.launch"
* Zum Starten von pose_estimation, midi_translation und der Koordinatentransformation von Kamera zum Flansch des Roboters 
5. Python-Skript "camera_test.py"
* Zum Testen der angeschlossenen Kamera
6. Python-Skript "camera_calibration.py"
* Zur vorherigen Kalibrierung der verwendeten Kamera

# Anleitung bei erster Inbetriebnahme für Bewegung des UR10e
1. Vorbereitung des PCs und des UR10e mit Hilfe folgender Anleitung: https://github.com/UniversalRobots/Universal_Robots_ROS_Driver
2. Anschließen der Kamera an den PC
3. Kalibrierung der Kamera
* Aufnahme von mind. 10 Fotos eines Schachbrettes aus verschiedenen Distanzen und Winkeln
* Anpassen der Parameter unter "Chessboard dimensions" und des Pfades, unter dem die aufgenommenen Fotos gefunden werden können
4. Öffnen eines Terminals, Starten eines Roscores
5. Öffnen eines Terminals, Starten des Teibers für den UR10e mit folgender Zeile:
cd catkin_ws
source devel setup.bash
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.0.101 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml
6. Starten des Programmes "External Control" auf dem UR10e
7. Öffnen eines Terminals, Starten des Launch-Files:
cd midi_drummer
source devel setup.bash
roslaunch drummer drummer.launch


##### Arduino Code zur Steuerung der Bewegung des Greifers #####

# Vorbereitung
1. Anschließen des Arduinos per USB an den PC
2. Anschließen der Energieversorgung an den Arduino

Informationen zur Handhabung des Greifers sind zu finden unter https://gitlab.lrz.de/ge72wan/code_semesterarbeit.git