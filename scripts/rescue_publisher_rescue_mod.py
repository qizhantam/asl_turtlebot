#!/usr/bin/env python

from std_msgs.msg import String, Bool
# from asl_turtlebot.msg import RescueMessage
import rospy
import sys, select, termios, tty
import numpy as np

msg = """
Welcome, rescuer!
Tell us when you accept to rescue cats and dogs!
---------------------------
Press on r to rescue
Press c to quit
---------------------------
"""

possible_scenarios =["is stuck in a tree!","is being chased by an elephant!","is fighting with a hamster!",
                     "is destroying somebody's garden gnome!",
                     "is bored.", "is hungry!", "wants to be pet.", "looks evil.", "was in trouble but I forgot why."]

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def create_bob_message(action="RESCUE"):
    Msg = RescueMessage()
    Msg.action = action
    Msg.keys = []
    Msg.animal_classes = []
    Msg.animal_poses_x_list = []
    Msg.animal_poses_y_list = []
    Msg.current_x = 0.0
    Msg.current_y = 0.0
    return Msg



# What happens when Bob receives a message
def message_received(Msg):

    if len(Msg.animal_poses_x_list) != 0:
        Msg_Rescue = "Veggie has found plenty of animals that need to be rescued! " \
                     "Based on your criteria, you get to choose " \
                     "who you want to save next. You will need to press on a number, in between 0 and 9. \n Veggie: '' "

        our_position = np.array([Msg.current_x,Msg.current_y])
        n = len(Msg.animal_poses_x_list)
        for k in range(n):
            key = Msg.keys[k]
            animal_position = np.array([Msg.animal_poses_x_list[k], Msg.animal_poses_y_list[k]])
            distance = np.linalg.norm(our_position-animal_position)
            animal_class = Msg.animal_classes[k]
            Msg_Rescue += "\n "+str(key)+".    A "+animal_class + " "+possible_scenarios[key]
            Msg_Rescue += "This happened "+str(int(distance))+"m away from here."

        Msg_Rescue += "  ''"
    else:
         Msg_Rescue = "It looks like Veggie has no more animal to rescue."
    print Msg_Rescue
    return

if __name__ == '__main__':
    rospy.init_node('Bob', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    rospy.Subscriber('/tb3_to_Bob', RescueMessage, message_received)
    pub = rospy.Publisher('/Bob_to_tb3', RescueMessage, queue_size=10)
    pub_test = rospy.Publisher("ready_to_rescue", Bool, queue_size = 10)

    # Menu
    print(msg)
    settings = termios.tcgetattr(sys.stdin)

    while not rospy.is_shutdown():
        Detected_Key = getKey()
        if Detected_Key == "r":
            Msg = create_bob_message("RESCUE")
            rospy.loginfo("RESCUE")
            pub.publish(Msg)
            pub_test.publish(True)
        elif Detected_Key == "c":
            break
        elif Detected_Key.isdigit():
            rospy.loginfo("BOB CHOSE TO SAVE ANIMAL ", Detected_Key)
            Msg = create_bob_message(Detected_Key)
            pub.publish(Msg)
        rate.sleep()

    #termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
