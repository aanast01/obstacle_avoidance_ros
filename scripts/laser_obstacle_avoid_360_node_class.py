#!/usr/bin/env python3

from Avoider import Avoider

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped#ros msg that deals with moving the robot
from sensor_msgs.msg import LaserScan #ros msg that gets the laser scans


dronePose = PoseStamped()
rateHz = 1
flag=False

def pose_callback(pose):
	global dronePos,flag
	dronePos = pose
	flag=True
	
def main():
    global dronePos,rateHz
    vel = Twist()
    # Instanciate our avoider object
    avoider = Avoider(vel)
    # Initialize our node
    rospy.init_node("Laser_Obs_Avoid_node")
    # Subscribe to the "/scan" topic in order to read laser scans data from it
    rospy.Subscriber("/scan", LaserScan, avoider.indentify_regions)
    
    rospy.Subscriber("/red/pose", PoseStamped, pose_callback)
    #create our publisher that'll publish to the "/cmd_vel" topic
    pub = rospy.Publisher("/red/tracker/input_pose", PoseStamped, queue_size = 1)
    #ros will try to run this code 10 times/second
    rate = rospy.Rate(rateHz) #10Hz
    
    #keep running while the ros-master isn't shutdown
    while not rospy.is_shutdown():
    	if flag:
    	    	vel = avoider.avoid()
    	    	data = PoseStamped()
    	    	data.header.frame_id = 'red/base_link'
    	    	
    	    	
    	    	vel.angular.x = vel.angular.x * 1/rateHz
    	    	vel.angular.y = vel.angular.y * 1/rateHz
    	    	vel.angular.z = vel.angular.z * 1/rateHz
    	    	angle = math.sqrt(math.pow(vel.angular.x,2) + math.pow(vel.angular.y,2) + math.pow(vel.angular.z,2))
    	    	
    	    	if(angle>0):
    	    		data.pose.orientation.x = dronePos.pose.orientation.x + vel.angular.x * math.sin(angle/2)/angle
    	    		data.pose.orientation.y = dronePos.pose.orientation.y + vel.angular.y * math.sin(angle/2)/angle
    	    		data.pose.orientation.z = dronePos.pose.orientation.z + vel.angular.z * math.sin(angle/2)/angle
    	    		data.pose.orientation.w = dronePos.pose.orientation.w + math.cos(angle/2)
    	    	else:
    	    		data.pose.orientation.w = 1;
    	    	
    	    	
    	    	siny_cosp = 2 * (data.pose.orientation.w * data.pose.orientation.z + data.pose.orientation.x * data.pose.orientation.y);
    	    	cosy_cosp = 1 - 2 * (data.pose.orientation.y * data.pose.orientation.y + data.pose.orientation.z * data.pose.orientation.z);
    	    	yaw = math.atan2(siny_cosp, cosy_cosp);
    	    	
    	    	data.pose.position.x = dronePos.pose.position.x+ 1/rateHz * (vel.linear.x)# * math.cos(yaw))
    	    	data.pose.position.y = dronePos.pose.position.y+ 1/rateHz * (vel.linear.y)# * math.sin(yaw))
    	    	data.pose.position.z = dronePos.pose.position.z+ 1/rateHz * vel.linear.z
    	    	
    	    	pub.publish(data)
    	    	#print(vel)
    	    	#print(data)
    	    	#print(yaw)
    	    	rate.sleep()
    	    	if(data.pose.position.x >= 2):
    	    		data.pose.position.x = dronePos.pose.position.x
    	    		data.pose.position.y = 0
    	    		data.pose.position.z = dronePos.pose.position.z
    	    		pub.publish(data)
    	    		exit(0)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
