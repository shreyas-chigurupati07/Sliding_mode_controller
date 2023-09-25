#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin, asin
from turtle import position
import numpy as np
import math as m
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
class Quadrotor():
	def __init__(self):
		# publisher for rotor speeds
		self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
		# subscribe to Odometry topic
		self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",
		Odometry, self.odom_callback)
		self.t0 = None
		self.t = None
		self.t_series = []
		self.x_series = []
		self.y_series = []
		self.z_series = []
		self.mutex_lock_on = False
		rospy.on_shutdown(self.save_data)
		self.x_req = [0,0,0,0]
		self.y_req = [0,0,0,0]
		self.z_req = [0,0,0]
		self.w_max = 2618  # maximum rotary speed 
		self.w_min = 0
		global omega
		omega = 0

	def rpy_limits(self,angle):
		return (angle+np.pi)%(2*np.pi)-np.pi
	

	def traj_evaluate(self):
		t = self.t
		# P0 to P1 
		if (t < 5):
			qp1_d = np.array([0, 0, (6*(t**5))/3125 - (3*(t**4))/125 + (2*(t**3))/25])
			dqp1_d = np.array([0, 0, (6*(t**4))/625 - (12*(t**3))/125 + (6*(t**2))/25])
			ddqp1_d = np.array([0, 0, (24*(t**3))/625 - (36*(t**2))/125 + (12*t)/25])
			x_d = qp1_d[0]
			y_d = qp1_d[1]
			z_d = qp1_d[2]
			dx_d = dqp1_d[0]
			dy_d = dqp1_d[1]
			dz_d = dqp1_d[2]
			ddx_d = ddqp1_d[0]
			ddy_d = ddqp1_d[1]
			ddz_d = ddqp1_d[2]

		# P1 to P2 
		elif(t < 20):
			qp2_d = np.array([(2*(t**5))/253125 - (t**4)/2025 + (22*(t**3))/2025 - (8*(t**2))/81 + (32*t)/81 - 47/81, 0, 1])
			dqp2_d = np.array([(2*(t**4))/50625 - (4*(t**3))/2025 + (22*(t**2))/675 - (16*t)/81 + 32/81, 0, 0])
			ddqp2_d = np.array([(8*(t**3))/50625 - (4*(t**2))/675 + (44*t)/675 - 16/81, 0, 0])
			x_d = qp2_d[0]
			y_d = qp2_d[1]
			z_d = qp2_d[2]
			dx_d = dqp2_d[0]
			dy_d = dqp2_d[1]
			dz_d = dqp2_d[2]
			ddx_d = ddqp2_d[0]
			ddy_d = ddqp2_d[1]
			ddz_d = ddqp2_d[2]

		 # P2 to P3    
		elif(t < 35):
			qp3_d = np.array([1, (2*(t**5))/253125 - (11*(t**4))/10125 + (118*(t**3))/2025 - (616*(t**2))/405 + (1568*t)/81 - 7808/81, 1])
			dqp3_d = np.array([0, (2*(t**4))/50625 - (44*(t**3))/10125 + (118*(t**2))/675 - (1232*t)/405 + 1568/81, 0])
			ddqp3_d = np.array([0, (8*(t**3))/50625 - (44*(t**2))/3375 + (236*t)/675 - 1232/405, 0])
			x_d = qp3_d[0]
			y_d = qp3_d[1]
			z_d = qp3_d[2]
			dx_d = dqp3_d[0]
			dy_d = dqp3_d[1]
			dz_d = dqp3_d[2]
			ddx_d = ddqp3_d[0]
			ddy_d = ddqp3_d[1]
			ddz_d = ddqp3_d[2]

		# P3 to P4 
		elif(t < 50):
			qp4_d = np.array([- (2*(t**5))/253125 + (17*(t**4))/10125 - (286*(t**3))/2025 + (476*(t**2))/81 - (9800*t)/81 + 80000/81, 1, 1])
			dqp4_d = np.array([- (2*(t**4))/50625 + (68*(t**3))/10125 - (286*(t**2))/675 + (952*t)/81 - 9800/81, 0, 0])
			ddqp4_d = np.array([- (8*(t**3))/50625 + (68*(t**2))/3375 - (572*t)/675 + 952/81, 0, 0])
			x_d = qp4_d[0]
			y_d = qp4_d[1]
			z_d = qp4_d[2]
			dx_d = dqp4_d[0]
			dy_d = dqp4_d[1]
			dz_d = dqp4_d[2]
			ddx_d = ddqp4_d[0]
			ddy_d = ddqp4_d[1]
			ddz_d = ddqp4_d[2]


		# P4 to P5 
		elif(t <= 65):
			qp5_d = np.array([0, -(2*(t**5))/253125 + (23*(t**4))/10125 - (526*(t**3))/2025 + (1196*(t**2))/81 - (33800*t)/81 + 380081/81, 1])
			dqp5_d = np.array([0, -(2*(t**4))/50625 + (92*(t**3))/10125 - (526*(t**2))/675 + (2392*t)/81 - 33800/81, 0])
			ddqp5_d = np.array([0, -(8*(t**3))/50625 + (92*(t**2))/3375 - (1052*t)/675 + 2392/81, 0])
			x_d = qp5_d[0]
			y_d = qp5_d[1]
			z_d = qp5_d[2]
			dx_d = dqp5_d[0]
			dy_d = dqp5_d[1]
			dz_d = dqp5_d[2]
			ddx_d = ddqp5_d[0]
			ddy_d = ddqp5_d[1]
			ddz_d = ddqp5_d[2]

		else:
			x_d = 0
			y_d = 0
			z_d = 1
			dx_d = 0
			dy_d = 0
			dz_d = 0
			ddx_d = 0
			ddy_d = 0
			ddz_d = 0
		return x_d, y_d, z_d, dx_d, dy_d, dz_d, ddx_d, ddy_d, ddz_d			

	def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
		# obtain the desired values by evaluating the corresponding trajectories
		global omega
		# Physical parameters (given)
		m=27*1e-3
		g=9.8
		l=46*1e-3
		Ix=16.5710*1e-6
		Iy=16.5710*1e-6
		Iz=29.261652*1e-6
		Ip=12.65625*1e-8
		kf=1.28192*1e-8
		km=5.964552*1e-3
		x_d, y_d, z_d, dx_d, dy_d, dz_d, ddx_d, ddy_d, ddz_d = self.traj_evaluate()


		x=xyz[0,0]
		y=xyz[1,0]
		z=xyz[2,0]
		dx=xyz_dot[0,0]
		dy=xyz_dot[1,0]
		dz=xyz_dot[2,0]
		phi=rpy[0,0]
		theta=rpy[1,0]
		psi=rpy[2,0]
		dphi=rpy_dot[0,0]
		dtheta=rpy_dot[1,0]
		dpsi=rpy_dot[2,0]  


		# Tuning Parameters             
		Kp = [80, 80] #[80 120];
		Kd = [18, 18]  #[1 10];
		K = np.array([13, 180, 150, 25]) #[5-15, 100-180, 100-180, 20-30];
		lamb = np.array([5, 15, 13, 7]) #[1-10, 10-15, 10-15, 1-10]
		Phi = 0.9  # Boundary layer

		# Forces realtions (given) to calculate phi_d & theta_d
		Fx = m * (-Kp[0]*(x-x_d) - Kd[0]*(dx-dx_d) + ddx_d)
		Fy = m * (-Kp[1]*(y-y_d) - Kd[1]*(dy-dy_d) + ddy_d)


		## Control Law
		# Control input u1
		e1 = z_d - z
		de1 = dz_d - dz
		S1 = de1 + (lamb[0]*e1)           # Sliding surface for u1
		sat1 = min(max(S1/Phi, -1), 1)    # Boundary condition for sliding surface S1
		u1 = (m * (g+ ddz_d + lamb[0]*de1 + K[0]*sat1)) /(cos(phi)*cos(theta))

		phi_d = asin(-Fy/u1)
		dphi_d = 0

		# Control input u2
		e2 =  self.rpy_limits(phi - phi_d)
		de2 = self.rpy_limits(dphi - dphi_d)
		S2 = de2 + (lamb[1]*e2)           # Sliding surface for u2
		sat2 = min(max(S2/Phi, -1), 1)    # Boundary condition for sliding surface S2
		u2 = (-dtheta*dpsi*(Iy-Iz)) + (Ip*omega*dtheta) + (((-lamb[1]*de2) - (K[1]*sat2))*Ix)

		theta_d = asin(Fx/u1)
		dtheta_d = 0

		# Control input u3
		e3 = self.rpy_limits(theta - theta_d)
		de3 = self.rpy_limits(dtheta - dtheta_d)
		S3 = de3 + (lamb[2]*e3)            # Sliding surface for u3
		sat3 = min(max(S3/Phi, -1), 1)     # Boundary condition for sliding surface S3
		u3 = (-dphi*dpsi*(Iz-Ix)) - (Ip*omega*dphi) - (lamb[2]*dtheta*Iy) - (K[2]*Iy*sat3)

		# Control input u4
		psi_d = 0
		dpsi_d =0
		e4 = self.rpy_limits(psi - psi_d)
		de4 = self.rpy_limits(dpsi - dpsi_d)
		S4 = de4 + (lamb[3]*e4)          # Sliding surface for u4         
		sat4 = min(max(S4/Phi, -1), 1)   # Boundary condition for sliding surface S4		
		u4 = (-dphi*dtheta*(Ix-Iy)) - (lamb[3]*dpsi*Iz) - (K[3]*Iz*sat4)
		
		u = np.array([[u1], [u2], [u3], [u4]])
		
		## Allocation Matrix
		A = np.array(([1/(4*kf), -sqrt(2)/(4*kf*l), -sqrt(2)/(4*kf*l),  -1/(4*km*kf)],
					  [1/(4*kf), -sqrt(2)/(4*kf*l),	 sqrt(2)/(4*kf*l), 	 1/(4*km*kf)],
					  [1/(4*kf),  sqrt(2)/(4*kf*l),	 sqrt(2)/(4*kf*l),  -1/(4*km*kf)],
					  [1/(4*kf),  sqrt(2)/(4*kf*l), -sqrt(2)/(4*kf*l),   1/(4*km*kf)]))

		w = np.matmul(A,u)
		motor_vel = np.zeros([4,1])
		motor_vel[0,0] = sqrt(w[0,0])
		motor_vel[1,0] = sqrt(w[1,0])
		motor_vel[2,0] = sqrt(w[2,0])
		motor_vel[3,0] = sqrt(w[3,0])
		
		# setting the limit of rotors velocities to 2618 rad/sec 
		for i in range(4):
			if (motor_vel[i,0] > self.w_max):
				motor_vel[i,0] = self.w_max

		omega = motor_vel[0,0]- motor_vel[1,0] + motor_vel[2,0]- motor_vel[3,0]

		motor_speed = Actuators()
		motor_speed.angular_velocities = [motor_vel[0,0], motor_vel[1,0],motor_vel[2,0], motor_vel[3,0]]
		self.motor_speed_pub.publish(motor_speed)


	# odometry callback function (DO NOT MODIFY)
	def odom_callback(self, msg):
		if self.t0 == None:
			self.t0 = msg.header.stamp.to_sec()
		self.t = msg.header.stamp.to_sec() - self.t0
		# convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
		w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.angular.y], [msg.twist.twist.angular.z]])
		v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.y], [msg.twist.twist.linear.z]])
		xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.y], [msg.pose.pose.position.z]])
		q = msg.pose.pose.orientation
		T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
		T[0:3, 3] = xyz[0:3, 0]
		R = T[0:3, 0:3]
		xyz_dot = np.dot(R, v_b)
		rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
		rpy_dot = np.dot(np.asarray([
			[1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
			[0, np.cos(rpy[0]), -np.sin(rpy[0])],
			[0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
			]), w_b)
		rpy = np.expand_dims(rpy, axis=1)
		# store the actual trajectory to be visualized later
		if (self.mutex_lock_on is not True):
			self.t_series.append(self.t)
			self.x_series.append(xyz[0, 0])
			self.y_series.append(xyz[1, 0])
			self.z_series.append(xyz[2, 0])
		# call the controller with the current states
		self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
		rospy.Rate(100)
	
	# save the actual trajectory data
	def save_data(self):
		with open("/home/shreyas/rbe502_project/src/project/scripts/log.pkl","wb") as fp:
			self.mutex_lock_on = True
			pickle.dump([self.t_series,self.x_series,self.y_series,self.z_series], fp)
		print("Plotting the trajectories...")
		os.system("rosrun project visualize.py")


if __name__ == '__main__':
	if os.path.exists("/home/shreyas/rbe502_project/src/project/scripts/log.pkl"):
		os.remove("/home/shreyas/rbe502_project/src/project/scripts/log.pkl")
	rospy.init_node("quadrotor_control")
	rospy.loginfo("Press Ctrl + C to terminate")
	whatever = Quadrotor()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
