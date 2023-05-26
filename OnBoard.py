import rclpy
from std_msgs.msg import Int8
import sys
import time
from threading import Lock
import os

uasID = 0
exitLock = True
debug = False
uasState = None
state = Int8()

# Command mapping for ground_station command topic
#  0 --> takeoff
#  1 --> land
#  2 --> emergency land


# Possible States for UAS:
#  0 --> uas on ground deactivated
#  1 --> uas on ground waiting command
#  2 --> uas in flight
#  3 --> uas in transition (landing or taking off)


# --------------------Command Functions--------------------------

def takeoff():
    state.data = 3
    statePub.publish(state)
    #take off procedure
    time.sleep(0.2)

    state.data = 2
    statePub.publish(state)


def land():
    state.data = 3
    statePub.publish(state)
    #take off procedure
    time.sleep(0.2)
    state.data = 1
    statePub.publish(state)


def emergency():
    global exitLock
    #emergency land procedure
    time.sleep(0.2)
    state.data = 0
    statePub.publish(state)
    rclpy.shutdown()




def cmdMonitor(msg):
    print('OnBoard received msg')
    keyMap[msg.data]()


keyMap = {0:takeoff, 1:land, 2:emergency}

# --------------------End Command Functions--------------------------


def main():
    global cmdSub, statePub
    rclpy.init(args=sys.argv)
    rosNode = rclpy.create_node(f'UAS{uasID}')
    statePub = rosNode.create_publisher(Int8, f'/UAS{uasID}/state', 0)
    cmdSub = rosNode.create_subscription(Int8, f'/control_station/UAS{uasID}/cmd', cmdMonitor, 0)
    
    time.sleep(10)
    
    state.data = 1
    statePub.publish(state)

    rclpy.spin(rosNode)

    time.sleep(0.5)

    #os.system('sudo shutdown now')




if __name__ == '__main__':
    if (len(sys.argv) != 1 and (sys.argv[1] == "--debug" or sys.argv[1] == "-debug")):
        debug = True

    main()