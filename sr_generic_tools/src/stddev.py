import roslib; roslib.load_manifest('sr_hand')
import rospy
import math
import string
import sys
import time
#from sr_robot_msgs.msg import  sensor_msgs
from sensor_msgs.msg import JointState
from sr_robot_msgs.msg import EthercatDebug
from std_msgs.msg import Float64, Int16, String


class EtherCAT_Debug_Listner:
    global sensors
    sensors = [ "FFJ1",  "FFJ2",  "FFJ3", "FFJ4", #0-3
                "MFJ1",  "MFJ2",  "MFJ3", "MFJ4", #4-7
                "RFJ1",  "RFJ2",  "RFJ3", "RFJ4", #8-11
                "LFJ1",  "LFJ2",  "LFJ3", "LFJ4", "LFJ5", #12-16
                "THJ1",  "THJ2",  "THJ3", "THJ4", "THJ5A", "THJ5B",#17-22
                "WRJ1A", "WRJ1B", "WRJ2",#23-25
                "ACCX",  "ACCY",  "ACCZ",
                "GYRX",  "GYRY",  "GYRZ"]#,
#                "AN0",   "AN1",   "AN2",  "AN3" ]
    def __init__(self, sensor_name):
        self.sensor_name_to_id = {}

        for i,v in enumerate(sensors) :
            self.sensor_name_to_id[v] = i

        self.values = []

        try :
            self.sensor_id = self.sensor_name_to_id[sensor_name.upper()]
        except:
            print "Bad sensor name"
            exit()

        self.debug_subscriber = rospy.Subscriber("/debug_etherCAT_data", EthercatDebug, self.debug_callback)

    def on_close(self):
        if self.debug_subscriber is not None:
            self.debug_subscriber.unregister()


        
    def sensor_get_name(self,id) :
        return sensors[id]

    def debug_callback(self, msg):

        if len(self.values) < 2000 : 
            if msg.sensors[self.sensor_id] != 0 :
                self.values.append(msg.sensors[self.sensor_id])

        else :

            mean = float(sum(self.values)) / float(len(self.values))
            mos = sum (float(x**2) for x in self.values)/float(len(self.values))
            std_dev = math.sqrt(sum ((x-mean)**2 for x in self.values) / (len(self.values)-1))

            print "mean is "+ str(mean)
            print "mean squared", mean*mean
            print"mos is ", mos
            print math.sqrt(mos - mean*mean)
            print "std dev is " + str(std_dev)
            self.on_close()

            exit()
        


def listner () :
    if len(sys.argv) == 1 :
        print "enter joint name as command line argument"
    else :
        EDL = EtherCAT_Debug_Listner(sys.argv[1])
        rospy.init_node('std_dev', anonymous=True)
        time.sleep (2.5)

listner()
