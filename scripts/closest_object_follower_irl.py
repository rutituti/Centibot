#!/usr/bin/env python3 
import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist 
import numpy as np

# This class will subscribe to the /base_scan topic and get the closest detected object
class ClosestObjectFollower(): 
    def __init__(self): 
        rospy.init_node("object_follower", anonymous=True) 
        rospy.on_shutdown(self.cleanup) 

        ############    SUBSCRIBERS   ####################### 
        rospy.Subscriber("scan", LaserScan, self.lidar_cb) 

        ############     PUBLISHERS ####################
        self.pub_cmd_vel = rospy.Publisher("cmd_vel",Twist, queue_size=1)

        ############ CONSTANTS AND VARIABLES ################ 
        self.lidar = LaserScan() #The data from the lidar will be kept here.
        self.d_closest = 0.0 #Distancia al objeto mas cercano detectado por el lidar
        self.theta_closest = 0.0 #Angulo del objeto mas cercano detectado por el lidar
        self.index = 0
        self.robot_vel = Twist()
        self.kw = 0.8 # Ganancia proporcional para el control de la velocidad angular
        self.kv = 0.3
         # Ganacia poporcional para el control de la velocidad linear
        self.w = 0.0 # Velocidad angular del robot
        self.v = 0.0 # Velocidad linear del robot

        self.turning_d = 0.6
        self.stop_d =  0.4

        r = rospy.Rate(10) #10Hz 
        print("Node initialized 1hz")
        
        while not rospy.is_shutdown(): 

            if self.lidar.ranges:
                
                print("Add some code to get the closest object distance and direction")
                self.d_closest = min(self.lidar.ranges)
                self.index = self.lidar.ranges.index(self.d_closest)
                self.theta_closest = self.lidar.angle_min + self.index*self.lidar.angle_increment
                self.theta_closest = np.arctan2(np.sin(self.theta_closest),np.cos(self.theta_closest)) # Get theta_closest from -pi to pi

                print("d_closest: ", self.d_closest)
                print("theta_closest",self.theta_closest)

                if (self.d_closest < self.turning_d and self.d_closest > self.stop_d):
                    self.w = self.kw*self.theta_closest
                    self.robot_vel.angular.z = self.w
                    self.robot_vel.linear.x = 0.0

                elif (self.d_closest < self.stop_d):
                    self.robot_vel.angular.z = 0.0
                    self.robot_vel.linear.x = 0.0
                else :
                    self.w = self.kw*self.theta_closest
                    self.robot_vel.angular.z = self.w

                    self.v = self.kv*self.d_closest
                    self.robot_vel.linear.x = self.v
                

                self.pub_cmd_vel.publish(self.robot_vel) # Publish de velocity value to the robot

            r.sleep()  #It is very important that the r.sleep function is called at least once every cycle. 
    
    def lidar_cb(self, lidar_msg): 
        ## This function receives the lidar message and copies this message to a member of the class 
        self.lidar = lidar_msg

        
        
    def cleanup(self): 
        #This function is called just before finishing the node 
        # You can use it to clean things up before leaving 
        # Example: stop the robot before finishing a node.
        stop_vel = Twist()
        self.pub_cmd_vel.publish(stop_vel)   
        print("I'm dying, bye bye!!!") 

############################### MAIN PROGRAM #################################### 
if __name__ == "__main__": 
    ClosestObjectFollower() 
