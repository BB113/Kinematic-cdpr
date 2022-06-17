#!/usr/bin/python3
import string
from xml.dom.minidom import parseString
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates

rospy.init_node('reading', anonymous=True)

def callback(data,name):
	fichier = open(name, "a+")
	fichier.write(str(data.position)+"\n")
	fichier.write(str(data.velocity)+"\n")
	fichier.write(str(data.effort)+"\n"+"\n")
	fichier.close()

def listener():
	#rospy.Subscriber("cable_command", JointState, callback,"cable_command")
	rospy.Subscriber("cable_states", JointState, callback, "cable_states")
	#rospy.Subscriber("desired_vel", JointState, callback, "desired_vel")
	#rospy.Subscriber("pf_states", ModelStates, callback, "pf_states")
	rospy.spin()
	
if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:    
            listener()

        except rospy.ROSInterruptException:
            pass
