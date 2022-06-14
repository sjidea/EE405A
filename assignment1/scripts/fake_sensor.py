#!/usr/bin/env python
# license removed for brevity
import rospy
# TODO: Import message types that you need (Float32)
from std_msgs.msg import String
from std_msgs.msg import Float32

class FakeSensor():
    def __init__(self):
        # Init ros node
        rospy.init_node('fake_sensor', anonymous=True)

        # TODO: Define publisher
	self.pub = rospy.Publisher('/vehicle_state', Float32, queue_size=10)

        # Define ros node rate
        self.rate = rospy.Rate(30) # 30hz

def main():
    # Create a class instance
    fake_sensor = FakeSensor()

    # Main loop
    while not rospy.is_shutdown():
        # TODO: Publish a fake sensor message as a Float32 message.
        #       (You can set the processed data arbitrary.)
	msg_vehicle_state_data = 3.19
	msg_vehicle_state = Float32()
	msg_vehicle_state.data = msg_vehicle_state_data
	fake_sensor.pub.publish(msg_vehicle_state)
	rospy.loginfo("I sent %f", msg_vehicle_state_data)

        # Rate control
        fake_sensor.rate.sleep()

if __name__ == '__main__':
    main()
