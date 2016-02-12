#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Range
import serial


def lightware_ros():
    rospy.init_node('lightware_ros')

    # ROS parameters
    hz = rospy.get_param('~rate', -1)
    port = rospy.get_param('~port', '/dev/ttyUSB0')
    t_out = rospy.get_param('~timeout', 0.05)
    baudrate = rospy.get_param('~baudrate', 115200)

    # init ROS stuff
    laser_pub = rospy.Publisher('range', Range, queue_size=10)

    if hz > 0:
        rate = rospy.Rate(hz)

    with serial.Serial(port, baudrate, timeout=t_out) as ser:
        while ser.isOpen() and not rospy.is_shutdown():
            ser.reset_input_buffer()
            payload = ser.readline()

            range_string = payload.split("m")[0]

            try:
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
            except ValueError:
                rospy.logwarn('Serial Read Error: %s', payload)

            if hz > 0:
                rate.sleep()


if __name__ == '__main__':
    try:
        lightware_ros()
    except rospy.ROSInterruptException:
        pass
