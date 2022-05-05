#!/usr/bin/env python3

import time
from pySerialTransfer import pySerialTransfer as txfer
import rospy
from std_msgs.msg import Float64MultiArray
import math

flag0 = 0
flag1 = 0

#Array mit den OCR1B- und OCR2B-Werten
#awrist =  [round(650*m.sin(i)+1500) for i in np.arange(0, 2*3.1415, 0.001)] #Sinusförmige Bewegung 
awrist = [1500, 6000] #Array für das Testen des Signals
athumb = [11, 80] #Array für das Testen des Signals
hittimes = None
starttime = None
hit_idx = math.nan

#Array Index
i1 = 0
i2 = 0

def wrist(pw1):
    #Schicke das nächste awrist Element
    send_size = 0
    send_size = link.tx_obj(pw1)
    link.send(send_size, 0)

    
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
    

def on_hittime_msg(msg):
    global starttime
    global hittimes
    global hit_idx
    starttime = rospy.Time.now()
    hittimes = msg.data
    hit_idx = 0


if __name__ == '__main__':
    #Init Ros
    rospy.init_node('servo_handler', anonymous=True)
    hittime_sub = rospy.Subscriber('hittime', Float64MultiArray, callback=on_hittime_msg)

    try:
        #Öffnen der Seriellen Schnittstelle
        link = txfer.SerialTransfer('/dev/ttyACM0')
        link.open()
        time.sleep(2)
        
        rate = rospy.Rate(200) 
        while not rospy.is_shutdown():
            # Initial value after new hittimes arrived
            if math.isnan(hit_idx):
                pass # do nothing
            
            elif link.available():
                #Falls neue Daten warten ausgelesen zu werden
                #Lese Daten
                data = link.rx_obj(obj_type = 'f')
                
                #Falls neue Daten für das Handgelenk angefordert werden
                if data == 1.0 or data == 2.0 :
                    if (rospy.Time.now() - starttime > hittimes[hit_idx]):
                        # execute a hit motion
                        wrist(6000)
                        thumb(80)
                        hit_idx += 1
                        if hit_idx > len(hittimes):
                            hit_idx = math.nan
                    else:
                        # set back to home
                        wrist(1500) 
                        thumb(11)
            
            rate.sleep()
                     
    except KeyboardInterrupt:
        link.close()
    
    except:
        import traceback
        traceback.print_exc()
        
        link.close()
