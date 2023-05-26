import rclpy
from pynput.keyboard import Key, Listener, KeyCode
from std_msgs.msg import Int8
import sys
import time
from threading import Lock

keyList = None
uasState = None
mutex = Lock()
cmd = Int8()

# Command mapping for ground_station command topic
#  0 --> takeoff
#  1 --> land
#  2 --> emergency land


# Possible States for UAS:
#  0 --> uas on ground deactivated
#  1 --> uas on ground waiting command
#  2 --> uas in flight
#  3 --> uas in transition (landing or taking off)

def stateMonitor(msg):
    global uasState
    print(f"UAS state changed to: {msg.data}")
    uasState = msg.data


# --------------------Command Functions--------------------------

def takeoff():
    print('~$Executed Command to Takeoff$~')
    if (uasState != 1):
        print("   -Drone not able to execute command\n")
        return

    mutex.acquire()
    cmd.data = 0
    cmdPub.publish(cmd)
    mutex.release()


def land():
    print('~$Executed Command to Land$~')
    if (uasState != 2):
        print("   -Drone not able to execute command\n")
        return
    
    mutex.acquire()
    cmd.data = 1
    cmdPub.publish(cmd)
    mutex.release()


def emergency():
    print('~$Executed Command for Emergency Landing$~')

    mutex.acquire()
    cmd.data = 2
    cmdPub.publish(cmd)
    mutex.release()

    time.sleep(0.5)

    keyList.stop()
    

keyMap = {KeyCode(char=' '):emergency, Key.esc:emergency, Key.enter:emergency, KeyCode(char='o'):takeoff, KeyCode(char='l'):land}

# --------------------End Command Functions--------------------------


def on_press(key):
    if (key in keyMap):
        keyMap[key]()

def on_release(key):
    pass


def print_commands():
    print("Avaliable Commands (where left is key and right is function):")
    for k, v in keyMap.items():
        print(f"{k} - {v.__name__}") 

def main():
    global cmdPub, stateSub
    print("Configuring ROS Node")
    keyList = Listener(on_press=on_press, on_release=on_release)
    rclpy.init(args=sys.argv)
    rosNode = rclpy.create_node('UAS_Station')
    cmdPub = rosNode.create_publisher(Int8, '/control_station/UAS0/cmd', 0)
    stateSub = rosNode.create_subscription(Int8, '/UAS0/state', stateMonitor, 0)
    time.sleep(0.5)


    input("Press enter to start executing")

    keyList.start()
    while (keyList.running):
        time.sleep(0.1)

    print("Exiting Program...")
    
    exit(1)


if __name__ == '__main__':
    print("Starting main()")
    main()