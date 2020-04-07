#!/usr/bin/env python
import roslib; roslib.load_manifest('motor_init')
import rospy
import tf.transformations
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
from time import sleep
def callback(msg):
    rospy.loginfo("Received a /cmd_vel message!")
    rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
    rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

    # Physical robot dimensions:
    L=0.22
    R=0.31
    v=msg.linear.x
    w=msg.angular.z

    # Do velocity processing here:
    # Use the kinematics of your robot to map linear and angular velocities into motor commands
    v_r = ((2 * v) + (w * L)) / (2 * R)
    v_l = ((2 * v) - (w * L)) / (2 * R)
    rospy.loginfo("Right and Left motorVelocity: [%f, %f]"%(v_r, v_l))

    # Then set your wheel speeds (using wheel_left and wheel_right as examples)
    en_b=22
    in1_b=27
    in2_b=17
    en=25
    in1=24
    in2=23

    GPIO.setmode(GPIO.BCM)
    rate = rospy.Rate(25)

    wheel_left_init(25,24,23,v_l)
    wheel_right_init(22,27,17,v_r)
    
    

def wheel_left_init(pwm_pin,in1,in2,power):
    
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    M = GPIO.PWM(pwm_pin, 1000)
    M.start(0) 

    convert_speed(pwm_pin,power)
    

def wheel_right_init(pwm_pin,in1,int2,power):
    GPIO.setup(in1, GPIO.OUT)
    GPIO.setup(in2, GPIO.OUT)
    M = GPIO.PWM(pwm_pin, 1000)
    M.start(0) 
    convert_speed(pwm_pin,power)


def convert_speed(pwm_pin,power):
    M = GPIO.PWM(pwm_pin, 1000)
    M.start(0) 

    W=power*100

    if W > 99.9:
            W = 99.9

    if W < 0.1:
            W = 0.1

    M.ChangeDutyCycle(W) 
             

    

        
        
   


def listener():
    rospy.init_node('cmd_vel_listener')
    rospy.Subscriber("/cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    while True:
        try:
            


            listener()
            


        except rospy.ROSInterruptException:
                        GPIO.cleanup()
                        print "closed"
                        pass