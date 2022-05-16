#!/usr/bin/env python3

import time
from pySerialTransfer import pySerialTransfer as txfer
import rospy
from std_msgs.msg import Float64MultiArray
import math

#Array with OCR1B and OCR2B values
#awrist =  [round(650*m.sin(i)+1500) for i in np.arange(0, 2*3.1415, 0.001)] # sinusoidal movement 
awrist = [1500, 6000] # Array for testing of the signal
athumb = [11, 80] # Array for testing of the signal
hittimes = None
starttime = None
hit_idx = math.nan

def wrist(pw1):
    """ Send next awrist element """
    send_size = 0
    send_size = link.tx_obj(pw1)
    link.send(send_size, 0)

    
def thumb(pw2):
    """ Send next athumb element """
    send_size = 0
    send_size = link.tx_obj(pw2)
    link.send(send_size, 1)

def on_hittime_msg(msg):
    """ Called on each trigger message for hitting """
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
        # Open serial interface
        link = txfer.SerialTransfer('/dev/ttyUSB0') # Set to the device which appears after plugging in the arduino
        link.open()
        time.sleep(2)
        
        rate = rospy.Rate(200) 
        while not rospy.is_shutdown():
            # Initial value after new hittimes arrived
            if math.isnan(hit_idx):
                pass # do nothing
            
            elif link.available():
                # If new data wait to be read
                # Read data
                data = link.rx_obj(obj_type = 'f')
                
                # If new data get requested for the wrist
                if data == 1.0 or data == 2.0 :
                    if (rospy.Time.now() - starttime > hittimes[hit_idx]):
                        # Execute a hit motion
                        wrist(6000)
                        thumb(80)
                        hit_idx += 1
                        if hit_idx > len(hittimes):
                            hit_idx = math.nan
                    else:
                        # Set back to home
                        wrist(1500) 
                        thumb(11)
            
            rate.sleep()
                     
    except KeyboardInterrupt:
        link.close()
    
    except:
        import traceback
        traceback.print_exc()
        
        link.close()
