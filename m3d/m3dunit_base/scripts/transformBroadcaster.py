#!/usr/bin/env python
import roslib
import rospy

import tf
import geometry_msgs.msg
from m3d_msgs.srv import calibration
import os
import json
TIM500_TRANSLATION = (0, 0.0035,  0);
LMS100_TRANSLATION = (0.074, 0, 0.068);

TIM500_ORIENTATION = (0, 0, 0, 1);
LMS100_ORIENTATION = (0, 0, 0, 1);

calibration_fileName = None

calibration_translation = (0,0,0)
calibration_orientation = (0,0,0,1)
def loadCalibrationFile():
    global calibration_translation
    global calibration_orientation
    matrix = None
    try:
        f = open(calibration_fileName, 'r');
        matrix = json.load(f);
        f.close()
    except :
        print "No calibration file under ", calibration_fileName
        f = open(calibration_fileName, 'w');
        matrix = [[0,0,0], [0,0,0,1]]
        json.dump(matrix, f)
        f.close()
    calibration_translation = matrix[0]
    calibration_orientation = matrix[1]
    return (matrix[0], matrix[1])

def saveCalibration (clb):
    global calibration_translation
    global calibration_orientation
    print "saving calibration", clb
    f = open(calibration_fileName, 'w');
    matrix = [[clb.offsetMatrix.translation.x,
               clb.offsetMatrix.translation.y,
               clb.offsetMatrix.translation.z],
             [clb.offsetMatrix.rotation.x,
             clb.offsetMatrix.rotation.y,
             clb.offsetMatrix.rotation.z,
             clb.offsetMatrix.rotation.w]]
    calibration_translation = matrix[0]
    calibration_orientation = matrix[1]

    json.dump(matrix, f)
    f.close()
    return "calibration file saved to " + calibration_fileName
if __name__ == '__main__':
    global calibration_fileName
    global calibration_translation
    global calibration_orientation
    calibration_fileName = os.environ.get('ROS_HOME');
    if calibration_fileName is None:
        calibration_fileName = os.environ.get('HOME')+"/.ros"
    calibration_fileName = calibration_fileName + "/m3d_calibration.yaml"

    print "calibration file is ", calibration_fileName
    loadCalibrationFile()


    rospy.init_node('laser_transform_broadcaster')
    front_laser_type = rospy.get_param('~front_laser_type', 'None');
    rot_laser_type   = rospy.get_param('~front_laser_type', 'None');
    prefix   = rospy.get_param('~prefix', '');
    service1 = rospy.Service('m3d_calibration', calibration, saveCalibration)

    r = rospy.Rate(10) # 10hz
    broadcaster = tf.TransformBroadcaster()

    while not rospy.is_shutdown():

        if rot_laser_type == "TIM500" :
            broadcaster.sendTransform(TIM500_TRANSLATION, TIM500_ORIENTATION,
                     rospy.Time.now(),
                     prefix+"rot_laser_optical", prefix+"m3d_rot_laser_link_uncalibrated")
        if rot_laser_type == "LMS100" :
            broadcaster.sendTransform(LMS100_TRANSLATION, LMS100_ORIENTATION,
                     rospy.Time.now(),
                     prefix+"m3d_rot_laser_link_uncalibrated", prefix+"rot_laser_optical")
        broadcaster.sendTransform(calibration_translation, calibration_orientation,
            rospy.Time.now(),
            prefix+"m3d_rot_laser_link_uncalibrated", prefix+"m3d_rot_laser_link")

        if front_laser_type == "TIM500" :
            broadcaster.sendTransform(TIM500_TRANSLATION, TIM500_ORIENTATION,
                     rospy.Time.now(),
                     prefix+"front_laser_optical",prefix+"m3d_front_laser_link")

        if front_laser_type == "LMS100" :
            broadcaster.sendTransform(TIM500_TRANSLATION, TIM500_ORIENTATION,
                     rospy.Time.now(),
                     prefix+"front_laser_optical",prefix+"m3d_front_laser_link")


        r.sleep()
