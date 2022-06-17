#!/usr/bin/python3
import string
from xml.dom.minidom import parseString
import rospy
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from multiprocessing import Pool


rospy.init_node('lds_distance', anonymous=True)
pool=Pool(4)

cc = open("cable_command", "a+")
cs = open("cable_states", "a+")
dv = open("desired_vel", "a+")
ms = open("model_states", "a+")

def reader(filename,data): 
	fichier = open(filename,"a+")
	fichier.write(str(data.position)+"\n")
	fichier.write(str(data.velocity)+"\n")
	fichier.write(str(data.effort)+"\n"+"\n")
	fichier.close()
	return 0


file_list = ["cable_command","cable_states","desired_vel","model_states"]
df_list = pool.map(reader, file_list)


def callback(data,fichier):
	print(fichier)
	fichier.write(str(data.position)+"\n")
	fichier.write(str(data.velocity)+"\n")
	fichier.write(str(data.effort)+"\n"+"\n")
	fichier.close()

def listener():
	global cc
	rospy.Subscriber("cable_command", JointState, callback, cc)
	rospy.Subscriber("cable_states", JointState, callback, cs)
	rospy.Subscriber("desired_vel", JointState, callback, dv)
	rospy.Subscriber("model_states", ModelStates, callback, ms)
	rospy.spin()
	
if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:    
            listener()

        except rospy.ROSInterruptException:
            cc.close()
            cs.close()
            dv.close()
            ms.close()
            pass
