# Arduino Code

## Anleitung

### Dateien
1. semesterarbeit_code/arduino/servo_control/servo_control.ino: 
Empfängt den gewünschten Servowinkel und erzeugt das richtige PWM-signal
2. semesterarbeit_code/servo_handler/script/servo_handler.py: 
Schickt auf Anfrage des Arduinos den neuen Wert für das OCR1B oder OCR2B Register.

### Vorbereitung
1. pySerialTransfer installieren: https://pypi.org/project/pySerialTransfer/2.1.6/
2. Dieses Repo in den catkin_ws clonen
3. Den catkin_ws bauen und sourcen: 
```
catkin_make
source ~/catkin_ws/devel/setup.bash
``` 

### Starten
1. Arduino per USB an den Computer anschließen
2. In der Arduino IDE den richtigen Port auswählen
3. In servo_handler.py den richtigen Port eintragen (zB: '/dev/ttyUSB0')
4. In servo_handler.py die Listcomprehension für die Sinusförmige Bewegung auskommentieren
5. Sketch  hochladen
6. In einem Terminal auf dem PC: 
```
roscore
```
7. In einem zweiten Terminal auf dem PC:
```
rosrun servo_handler servo_handler.py
```

### Arbeit mit der Arduino IDE und pySerialTransfer

pySerialTransfer und die Arduino IDE können den Serial Port nicht gleichzeitig benützen. 
Um ein neues Programm auf den Arduino aufzuspielen, muss servo_handler.py beendet werden.


Start driver for ur10e with calibration file 
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.0.101 kinematics_config:=$(rospack find ur_calibration)/my_robot_calibration.yaml