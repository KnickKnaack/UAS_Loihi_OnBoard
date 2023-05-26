import rclpy
from std_msgs.msg import Int8
import sys
import time
import os

uasID = 0
exitLock = True
debug = False
uasState = 1
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
    global uasState
    state.data = 3
    uasState = 3
    statePub.publish(state)
    #take off procedure
    time.sleep(0.2)

    state.data = 2
    uasState = 2
    statePub.publish(state)


def land():
    global uasState
    state.data = 3
    uasState = 3
    statePub.publish(state)
    #take off procedure
    time.sleep(0.2)
    state.data = 1
    uasState = 1
    statePub.publish(state)


def emergency():
    global exitLock, uasState
    #emergency land procedure
    time.sleep(0.2)
    state.data = 0
    uasState = 0
    statePub.publish(state)
    exit(1)


def init():
    #send current state
    state.data = uasState
    statePub.publish(state)



keyMap = {0:takeoff, 1:land, 2:emergency, 3:init}


def cmdMonitor(msg):
    print('OnBoard received msg')
    keyMap[msg.data]()


# --------------------End Command Functions--------------------------


def main():
    global cmdSub, statePub
    rclpy.init(args=sys.argv)
    rosNode = rclpy.create_node(f'UAS{uasID}')
    statePub = rosNode.create_publisher(Int8, f'/UAS{uasID}/state', 0)
    cmdSub = rosNode.create_subscription(Int8, f'/control_station/UAS{uasID}/cmd', cmdMonitor, 0)
    
    state.data = 1
    statePub.publish(state)

    rclpy.spin(rosNode)

    time.sleep(0.5)

    #os.system('sudo shutdown now')




if __name__ == '__main__':
    if (len(sys.argv) != 1 and (sys.argv[1] == "--debug" or sys.argv[1] == "-debug")):
        debug = True

    main()