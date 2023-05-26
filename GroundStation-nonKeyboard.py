import rclpy
from std_msgs.msg import Int8
import sys
import time
from threading import Lock

keyList = None
cmdPub = None
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
    cmdPub.publish(Int8)
    mutex.release()


def land():
    print('~$Executed Command to Land$~')
    if (uasState != 2):
        print("   -Drone not able to execute command\n")
        return
    
    mutex.acquire()
    cmd.data = 1
    cmdPub.publish(Int8)
    mutex.release()


def emergency():
    print('~$Executed Command for Emergency Landing$~')

    mutex.acquire()
    cmd.data = 2
    cmdPub.publish(Int8)
    mutex.release()

    time.sleep(0.5)

    keyList.stop()
    

keyMap = {'e':emergency, 'o':takeoff, 'l':land}

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
    print("Configuring ROS Node")
    rclpy.init(args=sys.argv)
    rosNode = rclpy.create_node('UAS_Station')
    cmdPub = rosNode.create_publisher(Int8, '/control_station/UAS0/cmd', 0)
    stateSub = rosNode.create_subscription(Int8, '/UAS0/state', stateMonitor, 0)
    time.sleep(0.5)


    input("Press enter to start executing")

    while (uIn != 'exit'):
        uIn = input("Enter Command: ")

        if uIn not in keyMap:
            print("Command not avaliable")
            continue
        
        keyMap[uIn]

    print("Exiting Program...")
    
    exit(1)


if __name__ == '__main__':
    print("Starting main()")
    main()