#!/usr/bin/env python
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

isButtonPressed = False;

buttonNo =1

def JoyCallback(data):
    global isButtonPressed
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.buttons)
    isButtonPressed= data.buttons[buttonNo];
    if isButtonPressed == True:
           rospy.logdebug(rospy.get_caller_id() + "Button!")
def joyListener():

    global buttonNo;
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/joy", Joy, JoyCallback)
    pub = rospy.Publisher('~button', Bool, queue_size=10)
    buttonNo = rospy.get_param('~buttonNo', 1)
    
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():

        pub.publish(isButtonPressed)
        rate.sleep()


if __name__ == '__main__':
    joyListener()

