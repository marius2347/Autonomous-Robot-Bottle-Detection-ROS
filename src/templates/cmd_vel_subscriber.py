#!/usr/bin/env python3
import sys, time, os, random, rospy, pyfirmata2, serial, threading
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

# ————————— USER-TUNABLE CONSTANTS —————————
L                         = 0.5
ARDUINO_PORT              = '/dev/ttyACM0' if os.path.exists('/dev/ttyACM0') else '/dev/ttyACM1'
obstacle_threshold        = 20.0      # cm
avoid_speed               = 0.3       # PWM (0–1)
backup_distance           = 0.8       # m
TURN_SEC                  = 3.0       # sec
STOP_SEC                  = 0.5       # sec
FORWARD_AFTER_TURN_SEC    = 5.0       # sec
CMD_VEL_TIMEOUT           = 1.0       # sec
AVOIDANCE_COOLDOWN_SEC    = 5.0       # sec
# ————————————————————————————————————————————

# ————— GLOBAL STATE + LOCKS —————
last_front      = float('inf')
last_right      = float('inf')
last_left       = float('inf')
obstacle_active = False
last_cmd_time   = time.time()
last_avoidance_time = 0

lock_front = threading.Lock()
lock_right = threading.Lock()
lock_left = threading.Lock()
lock_state = threading.Lock()
lock_cmd   = threading.Lock()
# ——————————————————————————————————

def reset_arduino(port):
    ser = serial.Serial(port, 9600)
    ser.dtr = False; time.sleep(1); ser.dtr = True; ser.close()

def set_both_wheels(left_speed, right_speed):
    M1_D1.write(int(left_speed > 0))
    M1_D2.write(int(left_speed < 0))
    M1_PWM.write(min(abs(left_speed), 1.0))

    M2_D1.write(int(right_speed > 0))
    M2_D2.write(int(right_speed < 0))
    M2_PWM.write(min(abs(right_speed), 1.0))

def stop():
    set_both_wheels(0, 0)

def reverse(sec):
    rospy.loginfo("REVERSING for %.2f s", sec)
    set_both_wheels(-avoid_speed, -avoid_speed)
    time.sleep(sec)
    stop()
    time.sleep(STOP_SEC)

def turn_left(sec=TURN_SEC):
    rospy.loginfo("TURNING LEFT for %.2f s", sec)
    set_both_wheels(-avoid_speed, avoid_speed)
    time.sleep(sec)
    stop()
    time.sleep(STOP_SEC)

def turn_right(sec=TURN_SEC):
    rospy.loginfo("TURNING RIGHT for %.2f s", sec)
    set_both_wheels(avoid_speed, -avoid_speed)
    time.sleep(sec)
    stop()
    time.sleep(STOP_SEC)

def drive_forward(sec=FORWARD_AFTER_TURN_SEC):
    rospy.loginfo("DRIVING FORWARD for %.2f s", sec)
    set_both_wheels(avoid_speed, avoid_speed)
    time.sleep(sec)
    stop()
    time.sleep(STOP_SEC)

def perform_avoidance():
    global obstacle_active, last_avoidance_time
    with lock_state:
        obstacle_active = True

    rospy.loginfo("==== AVOIDANCE START ====")
    stop()
    time.sleep(STOP_SEC)

    backup_sec = backup_distance / avoid_speed
    rospy.loginfo("Step: BACKUP %.2f m → %.2f s", backup_distance, backup_sec)
    reverse(backup_sec)

    with lock_left:  left  = last_left
    with lock_right: right = last_right
    rospy.loginfo("Step: CHECK sides – L=%.2f cm | R=%.2f cm", left, right)

    if left > obstacle_threshold:
        rospy.loginfo("Step: TURN LEFT")
        turn_left()
    elif right > obstacle_threshold:
        rospy.loginfo("Step: TURN RIGHT")
        turn_right()
    else:
        choice = random.choice(['left','right'])
        rospy.loginfo("Both blocked → RANDOM %s", choice)
        turn_left() if choice == 'left' else turn_right()

    rospy.loginfo("Step: FORWARD after turn")
    drive_forward()

    with lock_state:
        obstacle_active = False
        last_avoidance_time = time.time()
    rospy.loginfo("==== AVOIDANCE DONE ====")

def prox_front_cb(msg):
    global last_front
    v = msg.data
    with lock_front:
        last_front = v
    rospy.loginfo("Front: %.2f cm", v)

    now = time.time()
    with lock_state:
        if (not obstacle_active and v < obstacle_threshold and
                now - last_avoidance_time > AVOIDANCE_COOLDOWN_SEC):
            rospy.loginfo("Obstacle detected → triggering avoidance")
            threading.Thread(target=perform_avoidance, daemon=True).start()

def prox_right_cb(msg):
    global last_right
    with lock_right:
        last_right = msg.data
    rospy.loginfo("Right: %.2f cm", last_right)

def prox_left_cb(msg):
    global last_left
    with lock_left:
        last_left = msg.data
    rospy.loginfo("Left: %.2f cm", last_left)

def cmd_vel_cb(msg):
    global last_cmd_time
    with lock_cmd:
        last_cmd_time = time.time()

    with lock_state:
        if obstacle_active:
            rospy.loginfo("Ignoring cmd_vel – in avoidance")
            return

    lin = msg.linear.x
    ang = msg.angular.z

    if lin == 0 and ang == 0:
        rospy.loginfo("cmd_vel zero → STOP")
        stop()
        return

    lspd = lin - ang * L / 2
    rspd = lin + ang * L / 2
    rospy.loginfo("cmd_vel → L=%.2f R=%.2f", lspd, rspd)
    set_both_wheels(lspd, rspd)

def monitor_cmd_vel():
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        with lock_cmd:
            if time.time() - last_cmd_time > CMD_VEL_TIMEOUT:
                rospy.loginfo("cmd_vel timeout → STOP")
                stop()
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_motor_control')

    reset_arduino(ARDUINO_PORT)
    board = pyfirmata2.Arduino(ARDUINO_PORT)
    M1_PWM = board.get_pin('d:9:p');  M1_D1 = board.get_pin('d:2:o'); M1_D2 = board.get_pin('d:4:o')
    M2_PWM = board.get_pin('d:10:p'); M2_D1 = board.get_pin('d:7:o'); M2_D2 = board.get_pin('d:8:o')
    board.samplingOn()

    rospy.Subscriber('/sensor/proximity1', Float32, prox_front_cb)
    rospy.Subscriber('/sensor/proximity2', Float32, prox_right_cb)
    rospy.Subscriber('/sensor/proximity3', Float32, prox_left_cb)
    rospy.Subscriber('/cmd_vel', Twist, cmd_vel_cb)

    threading.Thread(target=monitor_cmd_vel, daemon=True).start()

    rospy.loginfo("Node ready – thresh=%.2f cm, backup=%.2f m", obstacle_threshold, backup_distance)
    rospy.spin()
