#!/usr/bin/env python3
import math
class Avoider():
	''' This class provides simple obstacle avoidance functionalities to a ROS robot '''

	# This dict keeps track of the distance measures for each region
	Regions_Report = {
	                     "front_L": [], "front_C": [], "front_R": [],
	                 }
	# These are the costs to deviate from each region to the goal region (front_C)
	Regions_Distances = {
	                     "front_L":  1, "front_C":  0, "front_R": -1,
	                 	}

	def __init__(self, vel_obj, obstacle_threshold=2.1, 
				       regional_angle=30, normal_lin_vel=1.0, 
				       trans_lin_vel=4.0, trans_ang_vel=12.0):
		'''
		:param vel_obj           : Velocity object; will contain velocity commands(data); Twist()
		:param obstacle_threshold: Objects at this distance or below are considered obstacles
		:param regional_angle    : The angle on which each region extends
		:param normal_lin_vel    : When there's no obstacles, the robot will move with this linear velocity
		:param trans_lin_vel     : After detecing an obstacle, the robot will back up (negative) 
								   while rotating to help in case it can't perform a stationary rotation
		:param trans_ang_vel 	 : The robot always rotates with the same value of angular velocity
		'''
		self.vel_obj        = vel_obj
		self.OBSTACLE_DIST  = obstacle_threshold
		self.REGIONAL_ANGLE = regional_angle
		self.NORMAL_LIN_VEL = normal_lin_vel
		self.TRANS_LIN_VEL  = trans_lin_vel
		self.TRANS_ANG_VEL  = trans_ang_vel

	def indentify_regions(self, scan):
		'''
		:param scan: Scan object that contains the lidar data 
		'''

		# This list keeps track of the order in which the regions' readings are obtained
		REGIONS = ["front_L", "front_C", "front_R",]

		# The front central region necessitate getting the last and first 15 points of the ranges
		#intermediary = scan.ranges[:int(self.REGIONAL_ANGLE/2)] + scan.ranges[(len(scan.ranges)-1)*int(self.REGIONAL_ANGLE/2):]
		#self.Regions_Report["front_C"] = [x for x in intermediary if x <= self.OBSTACLE_DIST and x != 'inf']
		
		self.Regions_Report["front_L"] = scan.ranges[-1:2*(int(len(scan.ranges)/len(REGIONS))):-1]
		#print(self.Regions_Report["front_L"])
		
		self.Regions_Report["front_C"] = scan.ranges[2*(int(len(scan.ranges)/len(REGIONS)))-1:int(len(scan.ranges)/len(REGIONS)):-1]
		#print(self.Regions_Report["front_C"])
		
		self.Regions_Report["front_R"] = scan.ranges[int(len(scan.ranges)/len(REGIONS))-1:0:-1]
		#print(self.Regions_Report["front_R"])
		
		#print(intermediary)
		# Enumerate all regions but the first
		for i, region in enumerate(REGIONS[0:]):
			# Only objects at a distance less than or equal to the threshold are considered obstacles
			#self.Regions_Report[region] = [x for x in scan.ranges[self.REGIONAL_ANGLE*i:self.REGIONAL_ANGLE*(i+1)]\
												   #if x <= self.OBSTACLE_DIST and x != 'inf']
			self.Regions_Report[region] = [x for x in self.Regions_Report[region] if x <= self.OBSTACLE_DIST and x != 'inf']
		#print(self.Regions_Report)

	def avoid(self):
		act, ang_vel = self._clearance_test()
		# If act is False, and_vel is set to 0
		self._steer(act, (act*ang_vel))
		return self.vel_obj

	def _clearance_test(self):

		goal = "front_C"
		closest = 10e6
		regional_dist = 0
		maxima = {"destination": "front_L", "distance": 10e-6, "max_length": 10e-6}
		for region in self.Regions_Report.items():
			#print('region: ', region[0])
			regional_dist = abs(self.Regions_Distances[region[0]]-self.Regions_Distances[goal])
			
			#if there're no obstacles in that region
			if not len(region[1]):
				#print('if')
				#check if it's the cheapest option
				if (regional_dist < closest):
					closest = regional_dist
					maxima["max_length"] = len(region[1])
					maxima["distance"] = self.OBSTACLE_DIST
					maxima["destination"] = region[0]
			#check if it's the clearest option
			elif((max(region[1]) > maxima["distance"])):
				#print('else')
				maxima["max_length"] = len(region[1])
				maxima["distance"] = max(region[1])
				maxima["destination"] = region[0]
		#calculate the cost to the chosen orientation
		regional_dist = self.Regions_Distances[maxima["destination"]]-self.Regions_Distances[goal]

		# Return whether to act or not, and the angular velocity with the appropriate sign
		return (closest != 0), (regional_dist/[abs(regional_dist) if regional_dist != 0 else 1][0])*self.TRANS_ANG_VEL

	def _steer(self, steer=False, ang_vel=0):
		'''
		:param steer  : Whether to avoid and obstacle or keep on going straigt
		:param ang_vel: The angular velocity of the robot
		'''
		if not steer:
			self.vel_obj.linear.x = self.NORMAL_LIN_VEL
			self.vel_obj.linear.y = 0
		else:
			self.vel_obj.linear.x = self.TRANS_LIN_VEL * 0.3
			self.vel_obj.linear.y = self.TRANS_LIN_VEL * (ang_vel/abs(ang_vel))
		self.vel_obj.linear.z  = 0
		self.vel_obj.angular.x = 0
		self.vel_obj.angular.y = 0
		self.vel_obj.angular.z = ang_vel
