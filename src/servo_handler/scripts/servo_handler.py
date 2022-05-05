#!/usr/bin/env python3

from re import M
import time
from pySerialTransfer import pySerialTransfer as txfer
import rospy
import numpy as np
import math as m


flag0 = 0
flag1 = 0

#Array mit den OCR1B- und OCR2B-Werten
#awrist =  [round(1000*m.sin(i)+3000) for i in np.arange(0, 2*3.1415, 0.001)] #Sinusförmige Bewegung 
awrist = [3000, 3000] #Array für das Testen des Signals
athumb = [11, 80] #Array für das Testen des Signals

#Array Index
i1 = 0
i2 = 0

#Init Ros
rospy.init_node('servo_handler', anonymous=True)
rate = rospy.Rate(200) 

def wrist(pw1):
    #Schicke das nächste awrist Element
    send_size = 0
    send_size = link.tx_obj(pw1)
    link.send(send_size, 0)
    print(pw1)

    
def thumb(pw2):
    #Schicke das nächste athumb Element
    send_size = 0
    send_size = link.tx_obj(pw2)
    link.send(send_size, 1)
    
    
def increment(index, array):
    #Imkrementiere den Index. Falls am Ende des Arrays angekommen, gehe an den Anfang
    if index < len(array) - 1:
        index = index + 1
    else: 
        index = 0
    return index
    

if __name__ == '__main__':

    try:
        #Öffnen der Seriellen Schnittstelle
        link = txfer.SerialTransfer('/dev/ttyUSB0')
        link.open()
        time.sleep(2)
        
        while not rospy.is_shutdown():
            if link.available():
                #Falls neue Daten warten ausgelesen zu werden
                #Lese Daten
                data = link.rx_obj(obj_type = 'f')
                
                #Falls neue Daten für das Handgelenk angefordert werden
                if data == 1.0:
                    wrist(awrist[i1])
                    i1 = increment(i1, awrist)

                #Falls neue Daten für den Daumen angefordert werden
                if data == 2.0:
                    thumb(athumb[i2])
                    i2 = increment(i2, athumb)
            
            rate.sleep()
                     
    except KeyboardInterrupt:
        link.close()
    
    except:
        import traceback
        traceback.print_exc()
        
        link.close()
