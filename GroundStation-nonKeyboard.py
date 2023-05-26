import rclpy
from std_msgs.msg import Int8
import sys
import time
from threading import Lock, Thread

keyList = None
uasState = 1
mutex = Lock()
cmd = Int8()

# Command mapping for ground_station command topic
#  0 --> takeoff
#  1 --> land
#  2 --> emergency land
#  3 --> request for current state of drone


# Possible States for UAS:
#  0 --> uas on ground deactivated
#  1 --> uas on ground waiting command
#  2 --> uas in flight
#  3 --> uas in transition (landing or taking off)

def stateMonitor(msg):
    global uasState
    #print(f"UAS state changed to: {msg.data}")
    uasState = msg.data


# --------------------Command Functions--------------------------

def takeoff():
    print('~$Executing Command to Takeoff$~')
    if (uasState != 1):
        print("   -Drone not able to execute command\n")
        return

    mutex.acquire()
    cmd.data = 0
    cmdPub.publish(cmd)
    print('published cmd from GS')
    mutex.release()


def land():
    print('~$Executing Command to Land$~')
    if (uasState != 2):
        print("   -Drone not able to execute command\n")
        return
    
    mutex.acquire()
    cmd.data = 1
    cmdPub.publish(cmd)
    mutex.release()


def emergency():
    print('~$Executing Command for Emergency Landing$~')

    mutex.acquire()
    cmd.data = 2
    cmdPub.publish(cmd)
    mutex.release()

    time.sleep(0.5)
    

keyMap = {'e':emergency, 'o':takeoff, 'l':land}

# --------------------End Command Functions--------------------------


def listen(args):
    rclpy.spin(args)

def print_commands():
    print("Avaliable Commands (where left is key and right is function):")
    for k, v in keyMap.items():
        print(f"{k} - {v.__name__}") 

def main():
    global cmdPub, stateSub
    print("Configuring ROS Node")

    #create ros node
    rclpy.init(args=sys.argv)
    rosNode = rclpy.create_node('UAS_Station')

    #create publisher for UAS commands
    cmdPub = rosNode.create_publisher(Int8, f'/control_station/UAS{0}/cmd', 0)
    #create subsctiber for UAS state
    stateSub = rosNode.create_subscription(Int8, f'/UAS{0}/state', stateMonitor, 0)
    
    #listener for state changes
    mylistener = Thread(target=listen, args=[rosNode])
    mylistener.start()
    
    
    time.sleep(0.5)

    #send initial request for the uas state
    cmd.data = 3
    cmdPub.publish(cmd)


    input("Press enter to start executing")
    uIn = None

    #loop to take user input
    while (uIn != 'exit'):
        uIn = input("Enter Command: ")

        if uIn not in keyMap:
            print("Command not avaliable")
            continue
        
        keyMap[uIn]()

    print("Exiting Program...")
    
    #shutdown for spinning Listener Thread
    rclpy.shutdown()

    exit(1)


if __name__ == '__main__':
    print("Starting main()")
    main()