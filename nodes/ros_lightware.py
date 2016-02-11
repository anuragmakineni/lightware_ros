#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
import serial


def ros_lightware():
    rospy.init_node('ros_lightware')

    # ROS parameters
    hz = rospy.get_param('~rate', 30)
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    t_out = rospy.get_param('~timeout', 0.05)
    baudrate = rospy.get_param('~baudrate', 115200)

    # init ROS stuff
    laser_pub = rospy.Publisher('range', Range, queue_size=10)
    rate = rospy.Rate(hz)

    try:
        ser = serial.Serial(port, baudrate, timeout=t_out)
    except serial.SerialException:
        rospy.logerr('Unable to connect to LightWare at %s:%d', port, baudrate)
        exit()

    while ser.isOpen():
        payload = ser.readline()

        range_string = payload[:8]  # get only the range portion of the string
        range_string = range_string.strip()

        if range_string == '---.--':
            distance = 0.0
        else:
            distance = float(range_string)

        # populate message
        msg = Range()
        msg.header.stamp = rospy.Time.now()
        msg.radiation_type = 1
        msg.field_of_view = 0.0035  # rad
        msg.min_range = 0.0  # m
        msg.max_range = 50.0  # m
        msg.range = distance

        laser_pub.publish(msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        ros_lightware()
    except rospy.ROSInterruptException:
        pass
