
import rospy
from sensor_msgs.msg import Joy


def main():
    rospy.init_node("joyTester")

    pub = rospy.Publisher("/red/joy", Joy, queue_size = 1)

    rospy.sleep(1)
    rate = rospy.Rate(10) #10Hz
    while not rospy.is_shutdown():
    	joy = Joy()
    	joy.axes.append(3)
    	pub.publish(joy)
    	rate.sleep()
	
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
