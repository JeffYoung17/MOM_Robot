#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

MYROBOT_MAX_LIN_VEL = 0.8
MYROBOT_MAX_ANG_VEL = 0.8
LIN_VEL_STEP_SIZE = 0.04
ANG_VEL_STEP_SIZE = 0.04 

msg = """
Control Your Mobile-Platform!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : +/- x_axis linear velocity (ROBOT : ~ 0.8m/s)
a/d : +/- y_axis linear velocity (ROBOT : ~ 0.8m/s)
q/e : +/- z_axis angular velocity (ROBOT : ~ 0.8rad/s)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel_x, target_linear_vel_y, target_angular_vel_z):
    return "currently:\tlinear_x vel %s\t linear_y vel %s\t angular vel %s " % (target_linear_vel_x, target_linear_vel_y, target_angular_vel_z)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -MYROBOT_MAX_LIN_VEL, MYROBOT_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -MYROBOT_MAX_ANG_VEL, MYROBOT_MAX_ANG_VEL)
    return vel

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('mecanum_platform_teleop')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    status = 0
    target_linear_vel_x   = 0.0
    target_linear_vel_y   = 0.0
    target_angular_vel_z  = 0.0
    control_linear_vel_x  = 0.0
    control_linear_vel_y  = 0.0
    control_angular_vel_z = 0.0

    try:
        print msg
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel_x = checkLinearLimitVelocity(target_linear_vel_x + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel_x, target_linear_vel_y, target_angular_vel_z)

            elif key == 'x' :
                target_linear_vel_x = checkLinearLimitVelocity(target_linear_vel_x - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel_x, target_linear_vel_y, target_angular_vel_z)

            elif key == 'a' :
                target_linear_vel_y = checkLinearLimitVelocity(target_linear_vel_y - LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel_y, target_linear_vel_y, target_angular_vel_z)

            elif key == 'd' :
                target_linear_vel_y = checkLinearLimitVelocity(target_linear_vel_y + LIN_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel_x, target_linear_vel_y, target_angular_vel_z)

            elif key == 'e' :
                target_angular_vel_z = checkAngularLimitVelocity(target_angular_vel_z - ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel_x, target_linear_vel_y, target_angular_vel_z)

            elif key == 'q' :
                target_angular_vel_z = checkAngularLimitVelocity(target_angular_vel_z + ANG_VEL_STEP_SIZE)
                status = status + 1
                print vels(target_linear_vel_x, target_linear_vel_y, target_angular_vel_z)

            elif key == ' ' or key == 's' :
                target_linear_vel_x   = 0.0
                control_linear_vel_x  = 0.0
                target_linear_vel_y   = 0.0
                control_linear_vel_y  = 0.0
                target_angular_vel_z  = 0.0
                control_angular_vel_z = 0.0
                print vels(target_linear_vel_x, target_linear_vel_y, target_angular_vel_z)
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print msg
                status = 0

            twist = Twist()

            control_linear_vel_x = makeSimpleProfile(control_linear_vel_x, target_linear_vel_x, (LIN_VEL_STEP_SIZE/2.0))
            control_linear_vel_y = makeSimpleProfile(control_linear_vel_y, target_linear_vel_y, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel_x; twist.linear.y = control_linear_vel_y; twist.linear.z = 0.0

            control_angular_vel_z = makeSimpleProfile(control_angular_vel_z, target_angular_vel_z, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel_z

            pub.publish(twist)

    except:
        print e

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
