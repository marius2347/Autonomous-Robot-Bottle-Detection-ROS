#!/usr/bin/env python3
import time, os, random, threading, rospy, pyfirmata2, serial
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ————— PARAMETERS —————
ARDUINO_PORT       = '/dev/ttyACM0' if os.path.exists('/dev/ttyACM0') else '/dev/ttyACM1'
OBSTACLE_THRESHOLD = 20.0   # cm
AVOID_SPEED        = 0.3    # PWM (0–1)
BACKUP_DISTANCE    = 0.3    # m
TURN_DURATION      = 3.0    # sec
FORWARD_DURATION   = 1.0    # sec
# ———————————————————

# ————— GLOBAL STATE —————
last_front = last_right = last_left = float('inf')
obstacle_active = False
board = None
M1_PWM = M1_D1 = M1_D2 = M2_PWM = M2_D1 = M2_D2 = None
# ——————————————————————

def reset_arduino(port):
    """Toggle DTR on Arduino to reset it."""
    try:
        ser = serial.Serial(port, 9600)
        ser.dtr = False; time.sleep(1)
        ser.dtr = True; ser.close()
        rospy.loginfo("Arduino reset.")
    except Exception as e:
        rospy.logwarn(f"Arduino reset failed: {e}")

def set_wheels(left, right):
    """Set left/right motor speeds (-1.0 to 1.0)."""
    M1_D1.write(left > 0); M1_D2.write(left < 0)
    M2_D1.write(right > 0); M2_D2.write(right < 0)
    M1_PWM.write(abs(left)); M2_PWM.write(abs(right))

def stop():
    """Stop both motors."""
    set_wheels(0, 0)

def perform_avoidance():
    """Backup, choose clear side, turn, then drive forward."""
    global obstacle_active
    obstacle_active = True
    stop(); time.sleep(0.5)

    # Backup
    bt = BACKUP_DISTANCE / AVOID_SPEED
    rospy.loginfo(f"Backing up for {bt:.2f}s")
    set_wheels(-AVOID_SPEED, -AVOID_SPEED); time.sleep(bt)
    stop(); time.sleep(0.5)

    # choose direction
    dir = 'left' if last_left > OBSTACLE_THRESHOLD else 'right'
    rospy.loginfo(f"Turning {dir}")
    if dir=='left':
        set_wheels(-AVOID_SPEED, AVOID_SPEED)
    else:
        set_wheels(AVOID_SPEED, -AVOID_SPEED)
    time.sleep(TURN_DURATION); stop(); time.sleep(0.5)

    # forward
    rospy.loginfo("Driving forward")
    set_wheels(AVOID_SPEED, AVOID_SPEED); time.sleep(FORWARD_DURATION)
    stop(); obstacle_active = False

def prox_front_cb(msg):
    """Front sensor: trigger avoidance if too close."""
    global last_front
    last_front = msg.data
    if not obstacle_active and last_front < OBSTACLE_THRESHOLD:
        rospy.loginfo("Front obstacle detected")
        perform_avoidance()

def prox_side_cb(msg, side):
    """Update left/right sensor readings."""
    globals()[f"last_{side}"] = msg.data

def cmd_vel_cb(msg):
    """Drive based on /cmd_vel, unless avoiding."""
    if obstacle_active:
        return
    lin, ang = msg.linear.x, msg.angular.z
    if lin==0 and ang==0:
        stop(); return
        
    # differential-drive kinematics (wheelbase = 0.5m)
    L = 0.5
    left  = lin - ang*L/2
    right = lin + ang*L/2
    set_wheels(left, right)

def exit_listener():
    """Wait for 'c'+Enter to shutdown and reset Arduino."""
    while not rospy.is_shutdown():
        if input().strip().lower() == 'c':
            rospy.loginfo("User requested shutdown")
            stop(); board.exit(); reset_arduino(ARDUINO_PORT)
            rospy.signal_shutdown("shutdown by user")

if __name__ == '__main__':
    rospy.init_node('motor_control')

    # start exit listener thread
    threading.Thread(target=exit_listener, daemon=True).start()

    # arduino init
    reset_arduino(ARDUINO_PORT)
    board = pyfirmata2.Arduino(ARDUINO_PORT)
    M1_PWM = board.get_pin('d:9:p');  M1_D1 = board.get_pin('d:2:o');  M1_D2 = board.get_pin('d:4:o')
    M2_PWM = board.get_pin('d:10:p'); M2_D1 = board.get_pin('d:7:o');  M2_D2 = board.get_pin('d:8:o')
    board.samplingOn()

    # subscribers
    rospy.Subscriber('/sensor/proximity1', Float32, prox_front_cb)
    rospy.Subscriber('/sensor/proximity2', Float32, lambda m: prox_side_cb(m,'right'))
    rospy.Subscriber('/sensor/proximity3', Float32, lambda m: prox_side_cb(m,'left'))
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)

    rospy.loginfo("Node ready")
    rospy.spin()
