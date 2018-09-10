#!/usr/bin/env python

from std_msgs.msg import String, Bool, Int16
import rospy
import sys, select, termios, tty

class RescuePublisher:
    def __init__(self):
        rospy.init_node('Bob', anonymous=True)
        self.pub = rospy.Publisher('/Bob_to_tb3', Int16, queue_size=10)
        self.pub_test = rospy.Publisher("ready_to_rescue", Bool, queue_size = 10)
        rospy.Subscriber('/tb3_to_Bob', String, self.tb3_message_received)
        self.menu_received = False

    def tb3_message_received(self, msg):
        # Prints menu of animals found from tb3
        print "Tell me the animal you want to rescue"
        print "-------------------------------------"
        print msg.data
        self.menu_received = True

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def loop(self):
        key = self.getKey()
        if key.isdigit() and self.menu_received:
            self.pub.publish(int(key))
            self.pub_test.publish(True)
            print "Go rescue animal {}".format(int(key))
            self.menu_received = False

    def run(self):
        print "Bob the rescuer here!"
        rate = rospy.Rate(10) # 10 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    res_pub = RescuePublisher()
    res_pub.run()
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
