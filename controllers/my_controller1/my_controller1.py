#~ /usr/bin/env python3.8

from controller import Robot, Motor
from math import radians, sqrt
import rospy
import threading
import std_msgs.msg as msg 
import time

TIME_STEP = 64

# create the Robot instance.
robot = Robot()


class DifferentialDriveRobot:
    def __init__(self):

        self.commands = []

        """ Robot specifications """
        self.w_max = 2.4               # [rad/s]
        self.wheel_radius = 66.0/2                 # [mm]
        self.v_max = self.w_max*self.wheel_radius  # [mm/s]
        self.L = 160                               # [mm]
                
        self.accel = 1*self.v_max

        self.state = 'idle'
        
        self.flag_teret = 0 #flag koji se podize kada robot pokupi teret
        
        """ Get and init all of the necesary sensors/actuators """
        self.left_wheel = robot.getDevice('left wheel motor')
        self.right_wheel = robot.getDevice('right wheel motor')
        
        self.left_wheel.setPosition(float('inf'))
        self.right_wheel.setPosition(float('inf'))
        
        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)
        
        self.left_gripper = robot.getDevice('left_gripper')
        self.right_gripper = robot.getDevice('right_gripper')
        
        
    def forward_motion(self, length_in_mm):
        if length_in_mm >= 0:
            sign = 1  
        else:
            sign = -1
            length_in_mm *= -1

        L_dist = length_in_mm
        v0 = 0
        v_ref = 0

        T1 = (self.v_max - 0) / self.accel
        L1 = 0 * T1 + self.accel * T1 * (T1 / 2)

        T3 = T1
        L3 = L1

        if (L1 + L3) <  L_dist:
            # can reach
            L2 = L_dist - L1 - L3
            T2 = L2 / self.v_max
            v_vrh = self.v_max
        else:
            L1 = L3 = L_dist/2
            # can't reach c_vmax
            T2 = 0
            T1 = T3 = sqrt(2*L1/self.accel)
            
            v_vrh = self.accel*T1

        t = t0 = robot.getTime()
        t1 = t0 + T1
        t2 = t1 + T2
        t3 = t2 + T3
        w_vrh = v_vrh/self.wheel_radius

        while t < t3:
            robot.step(TIME_STEP)
            t = robot.getTime()

            if(t <= t1):
                v_ref = v0 + self.accel * (t-t0)
            elif(t <= t2):
                v_ref = v_vrh
            elif(t <= t3):
                v_ref = v_vrh - self.accel * (t-t2)

            r_wheel_vel = v_ref/self.wheel_radius
            l_wheel_vel = r_wheel_vel

            self.left_wheel.setVelocity(sign*l_wheel_vel)
            self.right_wheel.setVelocity(sign*r_wheel_vel)

        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)
        
        
    def left_motion(self, length_in_mm):
        if length_in_mm >= 0:
            sign = 1  
        else:
            sign = -1
            length_in_mm *= -1

        L_dist = length_in_mm
        v0 = 0
        v_ref = 0

        T1 = (self.v_max - 0) / self.accel
        L1 = 0 * T1 + self.accel * T1 * (T1 / 2)

        T3 = T1
        L3 = L1

        if (L1 + L3) <  L_dist:
            # can reach
            L2 = L_dist - L1 - L3
            T2 = L2 / self.v_max
            v_vrh = self.v_max
        else:
            L1 = L3 = L_dist/2
            # can't reach c_vmax
            T2 = 0
            T1 = T3 = sqrt(2*L1/self.accel)

            v_vrh = self.accel*T1

        t = t0 = robot.getTime()
        t1 = t0 + T1
        t2 = t1 + T2
        t3 = t2 + T3
        w_vrh = v_vrh/self.wheel_radius

        while t < t3:
            robot.step(TIME_STEP)
            t = robot.getTime()

            if(t <= t1):
                v_ref = v0 + self.accel * (t-t0)
            elif(t <= t2):
                v_ref = v_vrh
            elif(t <= t3):
                v_ref = v_vrh - self.accel * (t-t2)

            r_wheel_vel = v_ref/self.wheel_radius
            l_wheel_vel = r_wheel_vel

            self.left_wheel.setVelocity(-l_wheel_vel)
            self.right_wheel.setVelocity(r_wheel_vel)


        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)
         
         
    def right_motion(self, length_in_mm):
        if length_in_mm >= 0:
            sign = 1  
        else:
            sign = -1
            length_in_mm *= -1

        L_dist = length_in_mm
        v0 = 0
        v_ref = 0

        T1 = (self.v_max - 0) / self.accel
        L1 = 0 * T1 + self.accel * T1 * (T1 / 2)

        T3 = T1
        L3 = L1

        if (L1 + L3) <  L_dist:
            # can reach
            L2 = L_dist - L1 - L3
            T2 = L2 / self.v_max
            v_vrh = self.v_max
        else:
            L1 = L3 = L_dist/2
            # can't reach c_vmax
            T2 = 0
            T1 = T3 = sqrt(2*L1/self.accel)

            v_vrh = self.accel*T1

        t = t0 = robot.getTime()
        t1 = t0 + T1
        t2 = t1 + T2
        t3 = t2 + T3
        w_vrh = v_vrh/self.wheel_radius

        while t < t3:
            robot.step(TIME_STEP)
            t = robot.getTime()

            if(t <= t1):
                v_ref = v0 + self.accel * (t-t0)
            elif(t <= t2):
                v_ref = v_vrh
            elif(t <= t3):
                v_ref = v_vrh - self.accel * (t-t2)

            r_wheel_vel = v_ref/self.wheel_radius
            l_wheel_vel = r_wheel_vel

            self.left_wheel.setVelocity(l_wheel_vel)
            self.right_wheel.setVelocity(-r_wheel_vel)


        self.left_wheel.setVelocity(0.0)
        self.right_wheel.setVelocity(0.0)  

# get the time step of the current world
timestep = int(robot.getBasicTimeStep())

myRobot = DifferentialDriveRobot()


# Mora novi thread jer rospy.spin() blokira task 
class WebotsThread(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.robot = robot
        
    def run(self):
        # perform simulation steps until webots is stopping the controller

        while robot.step(TIME_STEP) != -1:
                    if self.robot.commands:
                    # parse the string 
                        command_str = self.robot.commands.pop()
                        action = command_str[0]
                        value = int(command_str[1:])
            
                    
                        if action == 'f': 
                            print("idem pravo")
                            self.robot.forward_motion(value)
                            
                        elif action == 'l':
                            print("idem levo")
                            if(self.robot.flag_teret == 0):
                                self.robot.left_motion(124)
                            elif(self.robot.flag_teret == 1):
                                self.robot.left_motion(133)
                          
                            self.robot.forward_motion(value)
                           
                        elif action == 'r':
                            print("idem desno")
                            if(self.robot.flag_teret == 0):
                                self.robot.right_motion(124)
                            elif(self.robot.flag_teret == 1):
                                self.robot.right_motion(133)
                            
                            self.robot.forward_motion(value)
                            
                        elif action == 'p':
                            print("idem za 180") 
                            self.robot.right_motion(248) 
                         
                            self.robot.forward_motion(value)
                           
                        elif action  == 'g':
                            print("zatvaram gripper")
                            self.robot.flag_teret = 1 
                            rad = radians(value)
                            self.robot.right_gripper.setPosition(float(rad))
                            self.robot.left_gripper.setPosition(float(-rad))
                        elif action  == 'o':
                            print("otvaram gripper")
                            self.robot.flag_teret = 0 
                            rad = radians(value)
                            self.robot.right_gripper.setPosition(float(-rad))
                            self.robot.left_gripper.setPosition(float(rad))
                            print("udaljavam se")
                            self.robot.forward_motion(-350)
                          
                    pass
        
webots_thread = WebotsThread(myRobot)
webots_thread.start()

def sub_callback(data): 
   
    print("Callback", data) 
    myRobot.commands.append(data.data)
    
rospy.init_node('motion_driver')
rospy.Subscriber('turtlebot_motion', msg.String, sub_callback)
print("radii")

rospy.spin()