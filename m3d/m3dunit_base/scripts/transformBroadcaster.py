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
LMS100C_TRANSLATION = (0, 0, 0.068);
VLP16_TRANSLATION = (0, 0.0035,  0);


TIM500_ORIENTATION = (0, 0, 0, 1);
LMS100_ORIENTATION = (0, 0, 0, 1);
LMS100C_ORIENTATION = (0, 0, 0, 1);
VLP16_ORIENTATION = (0, 0, 0, 1);

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
    rot_laser_type   = rospy.get_param('~rot_laser_type', 'None');
    prefix   = rospy.get_param('~prefix', '');
    service1 = rospy.Service('m3d_calibration', calibration, saveCalibration)

    r = rospy.Rate(10) # 10hz
    broadcaster = tf.TransformBroadcaster()


    rotLaser_translation = None;
    rotLaser_orientation = None;

    frontLaser_translation = None;
    frontLaser_orientation = None;


    if rot_laser_type == "TIM500" :
        rotLaser_translation = TIM500_TRANSLATION;
        rotLaser_orientation = TIM500_ORIENTATION;
    if rot_laser_type == "LMS100" :
        rotLaser_translation = LMS100_TRANSLATION;
        rotLaser_orientation = LMS100_ORIENTATION;
    if rot_laser_type == "LMS100C" :
        rotLaser_translation = LMS100C_TRANSLATION;
        rotLaser_orientation = LMS100C_ORIENTATION;
    if rot_laser_type == "VLP16" :
        rotLaser_translation = VLP16_TRANSLATION;
        rotLaser_orientation = VLP16_ORIENTATION;

    if front_laser_type == "TIM500" :
        frontLaser_translation = TIM500_TRANSLATION;
        frontLaser_orientation = TIM500_ORIENTATION;
    if front_laser_type == "LMS100" :
        frontLaser_translation = LMS100_TRANSLATION;
        frontLaser_orientation = LMS100_ORIENTATION;
    if front_laser_type == "LMS100C" :
        frontLaser_translation = LMS100C_TRANSLATION;
        frontLaser_orientation = LMS100C_ORIENTATION;
    if front_laser_type == "VLP16" :
        frontLaser_translation = VLP16_TRANSLATION;
        frontLaser_orientation = VLP16_ORIENTATION;

    print "rot_laser_type         : ", rot_laser_type
    print "front_laser_type       : ", front_laser_type

    print "rotLaser_translation   : ", rotLaser_translation
    print "rotLaser_orientation   : ", rotLaser_orientation

    print "frontLaser_translation : ", frontLaser_translation
    print "frontLaser_orientation : ", frontLaser_orientation

    while not rospy.is_shutdown():
        if rotLaser_translation is not None:
            broadcaster.sendTransform(rotLaser_translation, rotLaser_orientation,
                     rospy.Time.now(),
                     prefix+"rot_laser_optical", prefix+"m3d_rot_laser_link_uncalibrated")

            broadcaster.sendTransform(calibration_translation, calibration_orientation,
                rospy.Time.now(),
                prefix+"m3d_rot_laser_link_uncalibrated", prefix+"m3d_rot_laser_link")

        if frontLaser_translation is not None :
            broadcaster.sendTransform(frontLaser_translation, frontLaser_orientation,
                     rospy.Time.now(),
                     prefix+"front_laser_optical",prefix+"m3d_front_laser_link")

        r.sleep()
