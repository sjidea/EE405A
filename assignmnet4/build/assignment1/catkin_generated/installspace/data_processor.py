#!/usr/bin/env python2
# license removed for brevity
import rospy
# TODO: Import message types that you need (Float32)
from std_msgs.msg import String
from std_msgs.msg import Float32

class StateProcessor():
    def __init__(self):
        # Init ros node
        rospy.init_node('processor', anonymous=True)

        # TODO: Define publisher and subscriber
        self.sub = rospy.Subscriber('/vehicle_state', Float32, self.callback_fake_sensor)
	self.processed = rospy.Publisher('/processed_state', Float32, queue_size=10)

        # Define ros node rate
        self.rate = rospy.Rate(100) # 100hz

    def callback_fake_sensor(self, msg):
        # TODO: Parse the data in the message
        msg_data = msg.data
	rospy.loginfo("I heard %f", msg_data)

        # TODO: Set an arbitrary output using the subscribed data in Float32
        #       and Publish the output as a Float32 message.
        #       (You can set the processed data arbitrary.)
	processed_msg_data = msg_data
	msg_processed = Float32()
	msg_processed.data = processed_msg_data
	self.processed.publish(msg_processed)

def main():
    # Create a class instance
    state_processor = StateProcessor()

    # Main loop
    while not rospy.is_shutdown():
        # Rate control
        state_processor.rate.sleep()

if __name__ == '__main__':
    main()
