### This file contains the node, that imports the model and the controller###
### It subscribes to the odometry, reference, clock, and other topics that come from the GUI ###
### It publishes to thrusters ###
# Creating a path to modules
import sys
import os
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(dir_path)
from module_rovModel import *
from module_rovController import *

#Python libraries and tools
import numpy as np
import logging
import csv
from datetime import datetime
import time
from random import gauss, randint
from casadi import *
import do_mpc

# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

class BluerovPubSubNode(Node):
    def __init__(self):
        super().__init__('bluerov_pubsub')
        self.ready_signal_mpc = False # Flag to start the MPC

        # Node launch parameters being declared
        self.declare_parameter('rov_id')
        self.declare_parameter('n_multi_agent')
        self.declare_parameter('FOV_constraint')
        self.declare_parameter('radius_setp')
        self.declare_parameter('distance_rovs')
        self.declare_parameter('FOV_range_deg')
        self.declare_parameter('FOV_range_soft_deg')
        self.declare_parameter('cycle_time_publish')

        # Extract and store parameters value
        self.main_id = self.get_parameter('rov_id').get_parameter_value().integer_value
        self.n_multi_agent = self.get_parameter('n_multi_agent').get_parameter_value().integer_value
        self.FOV_constraint = self.get_parameter('FOV_constraint').get_parameter_value().bool_value
        self.radius_setp = self.get_parameter('radius_setp').get_parameter_value().double_value
        self.distance_rovs = self.get_parameter('distance_rovs').get_parameter_value().double_value
        self.FOV_range_deg = self.get_parameter('FOV_range_deg').get_parameter_value().double_value
        self.FOV_range_soft_deg = self.get_parameter('FOV_range_soft_deg').get_parameter_value().double_value
        self.cycle_time_publish = self.get_parameter('cycle_time_publish').get_parameter_value().double_value
        
        #Creating variables
        self.angle2 = 0
        self.angle3 = 0
        self.record_data = 0
        self.dt_string = 0
        self.make_file = 0
        self.last_time = 0
        self.real_x2 = 0
        self.real_y2 = 0
        self.real_z2 = 0
        self.x2_to_mpc = 0
        self.y2_to_mpc = 0
        self.z2_to_mpc = 0

        # Initialize the MPC
        self.modelRov = MyROVModel()
        self.mpc1 = MyController(self.modelRov,                             # MPC-controller parameters
                                n_multi_agent=self.n_multi_agent,
                                radius_setp=self.radius_setp,               #Radius setpoint
                                distance_rovs=self.distance_rovs,           # Distance between ROVs
                                FOV_range_deg=self.FOV_range_deg,           # Hard constraint Field of View
                                FOV_range_soft_deg=self.FOV_range_soft_deg, # Soft constraint Field of View
                                FOV_constraint= self.FOV_constraint         # Activate or Deactivate constraint
                                )
        
        # Initialize subscribers for the main ROV
        self.main_odometry_subscriber = self.create_subscription(                 # Subscribe to odometry
            Odometry,                                                             # Message type
            "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(self.main_id), # Topic
            self.main_odemetry_callback,                                          # Callback function
            10)
        
        self.ref_subscriber = self.create_subscription(  # Subscribe to reference
            Vector3,
            "/ref",
            self.reference_callback, 
            10) 
        
        self.record_data_subscriber = self.create_subscription(  # Relevant for GUI
            Bool,
            "/record_data",
            self.record_data_callback, 
            10)


        #Creating publishers for the thrusters
        self.thruster1_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster1_joint/cmd_thrust'.format(self.main_id), 
            10)

        self.thruster2_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster2_joint/cmd_thrust'.format(self.main_id), 
            10)

        self.thruster3_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster3_joint/cmd_thrust'.format(self.main_id), 
            10)

        self.thruster4_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster4_joint/cmd_thrust'.format(self.main_id),
            10)

        self.thruster5_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster5_joint/cmd_thrust'.format(self.main_id), 
            10)

        self.thruster6_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster6_joint/cmd_thrust'.format(self.main_id), 
            10)

        self.thruster7_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster7_joint/cmd_thrust'.format(self.main_id), 
            10)

        self.thruster8_publisher = self.create_publisher(
            Float64, 
            '/model/bluerov{}/joint/thruster8_joint/cmd_thrust'.format(self.main_id), 
            10)

       
        multi_agent_id = [i for i in range(self.n_multi_agent)] #List of ROV IDs
        multi_agent_id.pop(self.main_id) #Remove the main ROV ID from the list

         # Subscribing to the odometry to the other ROVs in the fleet
        if(self.n_multi_agent > 1): 
            self.odometry_2_subscriber = self.create_subscription(  
                Odometry, # Message type
                "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(
                    multi_agent_id[self.main_id-1]), # Topic
                self.odometry_callback_2, # Callback function
                10)
                
            self.sec_rov = multi_agent_id[self.main_id-1]  ## TO BE REMOVED AT A LATER STAGE
            multi_agent_id.pop(self.main_id-1)
            self.angle_publisher = self.create_publisher( ## TO BE REMOVED AT A LATER STAGE
                Float64, # Message type
                'angle/from_{}_to_{}'.format(self.main_id, self.sec_rov), # Topic
                10) 
            
        if(self.n_multi_agent > 2):
            self.odometry_3_subscriber = self.create_subscription( 
                Odometry, # Message type
                "/bluerov2_pid/bluerov{}/observer/nlo/odom_ned".format(multi_agent_id[0]), # Topic
                self.odometry_callback_3, # Callback function
                10)
            
            self.third_rov = multi_agent_id[0]  ## TO BE REMOVED AT A LATER STAGE
            
            self.angle_publisher2 = self.create_publisher( ## TO BE REMOVED AT A LATER STAGE
                Float64, # Message Type
                'angle/from_{}_to_{}'.format(self.main_id, self.third_rov), # Topic
                10)

        self.main_odometry_subscriber # Prevent unused variable warning

        #Runs the publisher_callback function as a cycle
        self.timer = self.create_timer(self.cycle_time_publish, self.publisher_callback)

    def vector_between_rovs(self,x1,y1,z1,x2,y2,z2):
        """
        Gets xyz coords of ROVs as input. Returns the vector between them (- heave)
        """
        x = (x2-x1)
        y = (y2-y1)
        z = (z2-z1)
        return [x, y, z]
    
    def z_directional_vector_from_quaternion(self, q0, e1, e2, e3):
        """
        Returns the directional vector of the ROV in -z direction
        """
        x = -2*(e1*e3+e2*q0)
        y = -2*(e2*e3-e1*q0)
        z = -1+2*(e1**2+e2**2)
        return [x, y, z]
    
    def x_directional_vector_from_quaternion(self, q0, e1, e2, e3):
        """
        Returns the directional vector of the ROV in the x direction (surge)
        """
        x = 1-2*(e2**2+e3**2)
        y = 2*(e1*e2+e3*q0)
        z = 2*(e1*e3-e2*q0)
        return [x, y, z]

    def record_data_callback(self, msg):
        self.record_data = msg.data

    def reference_callback(self,msg):
        """
        Subscriber function for the reference topic
        """
        self.mpc1.x_setp = msg.x
        self.mpc1.y_setp = msg.y
        self.mpc1.z_setp = msg.z

        # CALCULATIING THE QUATERNION REFERENCES (Maybe move)
        if(self.ready_signal_mpc):
            if(self.FOV_constraint and self.n_multi_agent):
                comparison_vector = [self.mpc1.x_2, self.mpc1.y_2, self.mpc1.z_2]
            else:
                comparison_vector = [float(msg.x), float(msg.y), float(msg.z)]
            this_rov_pos = [float(self.x0[0]), float(self.x0[1]), float(self.x0[2])]
            vector = np.array(comparison_vector) - np.array(this_rov_pos)
            vector = vector / np.linalg.norm(vector)
            theta = np.arccos(np.dot([1, 0, 0], vector))
            axis = np.cross([1, 0, 0], vector)
            if(sum(axis) != 0):
                axis = axis / np.linalg.norm(axis)
                self.mpc1.q_0_setp = np.cos(theta/2)
                self.mpc1.e_1_setp = axis[0] * np.sin(theta/2)
                self.mpc1.e_2_setp = axis[1] * np.sin(theta/2)
                self.mpc1.e_3_setp = axis[2] * np.sin(theta/2)

    def main_odemetry_callback(self, msg):
        """
        Subscriber function for the main ROV odometry
        """
        self.odometry_list = [msg.pose.pose.position.x,
                              msg.pose.pose.position.y,
                              msg.pose.pose.position.z,
                              msg.pose.pose.orientation.w,
                              msg.pose.pose.orientation.x,
                              msg.pose.pose.orientation.y,
                              msg.pose.pose.orientation.z,
                              msg.twist.twist.linear.x,
                              msg.twist.twist.linear.y,
                              msg.twist.twist.linear.z,
                              msg.twist.twist.angular.x,
                              msg.twist.twist.angular.y,
                              msg.twist.twist.angular.z]
        self.x0 = np.array(self.odometry_list)
               
        if(not self.ready_signal_mpc): #First cycle
            self.mpc1.x0 = self.x0
            self.mpc1.mpc.set_initial_guess()
            self.ready_signal_mpc = True

        if(not self.record_data):
            now = datetime.now()
            self.dt_string = now.strftime("Date--%d--%m--%y--Time--%H--%M--%S--")
            self.make_file = 0
            
        #Writes relevant information to the CSV
        if(self.record_data and self.ready_signal_mpc):
            if(not self.make_file):
                with open(
                    (str('csv_data/'+self.dt_string) +'--rov{}.csv'.format(str(self.main_id))),'w') as f:
                    writer = csv.writer(f)
                    writer.writerow(
                        ['x_ref','y_ref','z_ref',
                         'x','y','z',
                         'eta','e1','e2','e3',
                         'u','v','w',
                         'p','q','r',
                         'angle2','angle3',
                         'x2', 'y2', 'z2', 
                         'real_x2', 'real_y2', 'real_z2', 
                         'time']
                         )
                    self.start_time = time.time()
                    self.make_file = 1
            
            with open(
                (str('csv_data/'+self.dt_string) +'--rov{}.csv'.format(str(self.main_id))), 'a') as f:
                writer = csv.writer(f)
                writer.writerow(
                    [self.mpc1.x_setp,self.mpc1.y_setp,self.mpc1.z_setp] + 
                    self.odometry_list + 
                    [self.angle2,self.angle3] + 
                    [self.x2_to_mpc, self.y2_to_mpc, self.z2_to_mpc] + 
                    [self.real_x2, self.real_y2, self.real_z2] + 
                    [time.time()-self.start_time]
                    )
    
    def odometry_callback_2(self, msg):
        """
        Subscriber function for 2nd ROV odometry
        """
        #if(randint(1, 10) <= 3): packetloss
        #dist_x = gauss(0, 0.5)  disturbance
        #dist_y = gauss(0, 0.5)
        #dist_z = gauss(0, 0.5)
        self.x2_to_mpc = msg.pose.pose.position.x# + dist_x
        self.y2_to_mpc = msg.pose.pose.position.y# + dist_y
        self.z2_to_mpc = msg.pose.pose.position.z# + dist_z
        self.mpc1.x_2 = msg.pose.pose.position.x # + dist_x
        self.mpc1.y_2 = msg.pose.pose.position.y # + dist_y
        self.mpc1.z_2 = msg.pose.pose.position.z # + dist_z
        self.real_x2 = msg.pose.pose.position.x
        self.real_y2 = msg.pose.pose.position.y
        self.real_z2 = msg.pose.pose.position.z
        #self.last_time = current_time
        if(self.ready_signal_mpc):
            v1 = self.vector_between_rovs(
                self.x0[0], self.x0[1], self.x0[2], 
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            
            v2 = self.x_directional_vector_from_quaternion(
                self.x0[3], self.x0[4], self.x0[5], self.x0[6])
            
            angle = Float64()
            self.angle2 = round(
                ((180*(np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))))/np.pi),
                2)
            
            angle.data = self.angle2
            self.angle_publisher.publish(angle)
            
    def odometry_callback_3(self, msg):
        """
        Subscriber function for 3rd ROV odometry
        """
        self.mpc1.x_3 = msg.pose.pose.position.x
        self.mpc1.y_3 = msg.pose.pose.position.y
        self.mpc1.z_3 = msg.pose.pose.position.z
        if(self.ready_signal_mpc):
            v1 = self.vector_between_rovs(
                self.x0[0], self.x0[1], self.x0[2], 
                msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
            
            v2 = self.x_directional_vector_from_quaternion(
                self.x0[3], self.x0[4], self.x0[5], self.x0[6])
            
            angle = Float64()
            self.angle3 = round(
                ((180*(np.arccos(np.dot(v1, v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))))/np.pi),
                2)
            
            angle.data = self.angle3
            self.angle_publisher2.publish(angle)
    

    def publisher_callback(self):
        """
        Running the MPC and publishing the thrusts
        """
        if(self.ready_signal_mpc): #If the odometry is ready

            # Calculate next step    
            self.u0_1 = self.mpc1.mpc.make_step(self.x0)

            # Publishing the thrusts
            thrust1 = Float64()
            thrust1.data = round(float(self.u0_1[0][0]),2)

            thrust2 = Float64()
            thrust2.data = round(float(self.u0_1[1][0]),2)

            thrust3 = Float64()
            thrust3.data = round(float(self.u0_1[2][0]),2)

            thrust4 = Float64()
            thrust4.data = round(float(self.u0_1[3][0]),2)

            thrust5 = Float64()
            thrust5.data = round(float(self.u0_1[4][0]),2)

            thrust6 = Float64()
            thrust6.data = round(float(self.u0_1[5][0]),2)

            thrust7 = Float64()
            thrust7.data = round(float(self.u0_1[6][0]),2)

            thrust8 = Float64()
            thrust8.data = round(float(self.u0_1[7][0]),2)
            

            self.thruster1_publisher.publish(thrust1)
            self.thruster2_publisher.publish(thrust2)
            self.thruster3_publisher.publish(thrust3)
            self.thruster4_publisher.publish(thrust4)
            self.thruster5_publisher.publish(thrust5)
            self.thruster6_publisher.publish(thrust6)
            self.thruster7_publisher.publish(thrust7)
            self.thruster8_publisher.publish(thrust8)


def main(args=None):
    rclpy.init(args=args)
    bluerov_pubsub_node = BluerovPubSubNode()
    rclpy.spin(bluerov_pubsub_node)
    rclpy.shutdown() 
main()
