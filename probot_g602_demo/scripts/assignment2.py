#!/usr/bin/env python

from itertools import combinations, product
from sys import exit
import os
import cv2
import sys
import numpy as np
import christofides

from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import Image as ImageMsg
from moveit_msgs.msg import VisibilityConstraint
from cv_bridge import CvBridge
import moveit_commander
import rospy
import math


# -----------------------------------Params-----------------------------------
INITIAL_SCAN_DISTANCE_PROPORTION = 0.5
ACCEPTABLE_CORRECTION = 0.5
FLOWER_LOCATION_TOLERANCE = 0.15
MAX_SCAN_ATTEMPTS = 5
ACCEPTABLE_PROXIMITY = 0.05
RADIUS_STEP_SIZE = 0.01
ROBOT_REACH_RADIUS = 0.6258  # Empirically tested
NUM_IMGS_PER_SCAN = 8
MIN_SUFFICIENT_VIEWPOINTS = 4  # Minimum of 2
MAX_OUTLIER_PERCENTAGE_TOLERANCE = 1/4
PLANT_LOCATION = Point(0.6, 0, 0.25)
ROBOT_BASE_LOCATION = (0, 0, 0)
CAMERA_TOPIC = "/usb_cam/camera1/image_raw"
CAMERA_LINK_NAME = "camera_link"
GRIPPER_LINK_NAME = "gripper_tip"
SHOW_IMGS = False

# -----------------------------------Error classes-----------------------------------
class FlowerNotFoundError(Exception):
	pass

# -----------------------------------Tom code-----------------------------------

latest_imgmsg = None

def stream_imagemsgs(msg):
	global latest_imgmsg
	latest_imgmsg = msg
		
class TomCodeClass:
	def __init__(self):
		with np.load(r"/home/tom/probot_g602_ws/src/probot_g602/probot_g602_demo/scripts/Camera_params.npz") as params:
			self.mtx = params['mtx']
			print("Loaded mtx:\n" + str(self.mtx))
		
		# Akiva's code until end of __init__
		self.img_converter = CvBridge()  
		rospy.Subscriber(CAMERA_TOPIC, ImageMsg, stream_imagemsgs)
		
	def lookAt(self, pos, target, up):
		pos = np.array(pos,dtype=np.float32); target = np.array(target, dtype=np.float32); up = np.array(up, dtype=np.float32)
		z = pos - target; z *= (1.0 / np.linalg.norm(z))
		x = np.cross(up, z); x *= (1.0/np.linalg.norm(x))
		y = np.cross(z,x)
		rot = np.vstack((x,y,z))
		#print(rot)
		vec=cv2.Rodrigues(rot)[0]
		#print(vec)
		return vec
		
	def euler_from_quaternion(self,x, y, z, w):
		#print('orientation: ',x,y,z,w)
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
		R=np.asarray([roll_x, pitch_y, yaw_z])
		#print('R: ',R)
		return  R # in radians
	
	def quaternion_from_euler(self, roll, pitch, yaw):
		"""
        Convert an Euler angle to a quaternion.

        Input
          :param roll: The roll (rotation around x-axis) angle in radians.
          :param pitch: The pitch (rotation around y-axis) angle in radians.
          :param yaw: The yaw (rotation around z-axis) angle in radians.

        Output
          :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
		qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
		qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
		qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
		qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2)
		
		return [qx, qy, qz, qw]
	
	def get3Dposition(self,im1P,im2P,f1P,f2P):
		# projection matrix
		# 1
		rotation_mat = np.zeros(shape=(3, 3))
		Rq1=self.euler_from_quaternion(f1P.orientation.x,f1P.orientation.y,f1P.orientation.z,f1P.orientation.w)
		R1 = cv2.Rodrigues(Rq1, rotation_mat)[0]
		T1= np.asarray([f1P.position.x,f1P.position.y,f1P.position.z]).T
		#print('R1:',R1)
		#print('T1:', T1)
		PA1 = np.concatenate((np.dot(self.mtx, R1), np.dot(self.mtx,T1[..., None])), axis=1)
		# 2
		rotation_mat = np.zeros(shape=(3, 3))
		Rq2 = self.euler_from_quaternion(f2P.orientation.x, f2P.orientation.y, f2P.orientation.z, f2P.orientation.w)
		R2 = cv2.Rodrigues(Rq2, rotation_mat)[0]
		PA2 = np.concatenate((np.dot(self.mtx, R2), np.dot(self.mtx, np.asarray([f2P.position.x, f2P.position.y, f2P.position.z])[..., None])), axis=1)
		points3D = cv2.triangulatePoints(PA1, PA2, im1P, im2P)
		points3D /= points3D[3]
		points3d = points3D.T[:, :3]
		#print('points3d\n', points3d)
		return points3d
		
	def snap_img(self, vid):
		# return vid.read()[1]
		# msg = rospy.wait_for_message(CAMERA_TOPIC, ImageMsg)
		return np.asarray(self.img_converter.imgmsg_to_cv2(latest_imgmsg))
	
	def get_flower_im_coor(self, vid):
		xp=-1
		yp=-1
		# ret, frame = self.vid.read()
		# cv2.imshow('frame', frame)
		# cv2.waitKey(50)
		detected = False
		# while (detected is not True):
		for i in range(5):
			# Take each frame
			img = self.snap_img(vid)
			
			# img = cv2.flip(img, 5)
			# Convert BGR to HSV
			hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

			# cv2.imshow("b", hsv)
			# define range of red color in HSV
			lower_color = np.array([40, 50, 130], dtype=np.uint8)
			upper_color = np.array([255, 255, 240], dtype=np.uint8)

			# Threshold the HSV image to get only blue colors
			mask = cv2.inRange(hsv, lower_color, upper_color)
			# cv2.imshow('mask',mask)
			# Bitwise-AND mask and original image
			res = cv2.bitwise_and(img, img, mask=mask)
			# cv2.imshow("b", res)

			imgray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

			ret, thresh = cv2.threshold(imgray, 100, 255, 0)
			im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			if len(contours)>0:
				contour_sizes = [(cv2.contourArea(contour), contour) for contour in contours]
				biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]

				x, y, w, h = cv2.boundingRect(biggest_contour)
				if w>15 or h>15:
					xp=int((x+w)/2)
					yp = int((y + h) / 2)
				cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
				# img = cv2.putText(img, 'P', (xp,yp), cv2.FONT_HERSHEY_SIMPLEX,
				#					 1, (255, 0, 0), 2, cv2.LINE_AA)
			# cv2.drawContours(img, contours, -1, (0,255,0), 3)
			if SHOW_IMGS:
				cv2.imshow('img', img)
				cv2.waitKey(2000)
				cv2.destroyAllWindows()


			k = cv2.waitKey(5) & 0xFF
			if k == 27:
				break
		if xp<0:
			#print('not detected')
			raise FlowerNotFoundError("flower not found in image")
		else:
			pass
			#print('Detected:',(xp,yp))
		return xp,yp
	
TomCode = TomCodeClass()

# -----------------------------------MoveIt config functions-----------------------------------
def init_moveit():
	print("Initializing moveit")
	moveit_commander.roscpp_initialize(sys.argv)
	rospy.init_node('flower_nudger')
	global arm
	arm = moveit_commander.MoveGroupCommander('manipulator')
	global gripper
	gripper = arm.set_end_effector_link(GRIPPER_LINK_NAME)
	gripper = arm.get_end_effector_link()
	
	reference_frame = 'base_link'
	arm.set_pose_reference_frame(reference_frame)
	arm.allow_replanning(True)
	arm.set_goal_position_tolerance(0.001)
	arm.set_goal_orientation_tolerance(0.001)
	arm.set_max_acceleration_scaling_factor(0.5)
	arm.set_max_velocity_scaling_factor(0.5)
	arm.set_named_target('home')
	arm.go()
	rospy.sleep(1)
	arm.set_start_state_to_current_state()

def destroy_moveit():
	print("Destroying moveit")
	moveit_commander.roscpp_shutdown()
	moveit_commander.os._exit(0)


# -----------------------------------Auxiliary functions-----------------------------------
def get_distance(point1, point2):
	if type(point1) != Point:
		point1 = Point(*point1.flatten())
	if type(point2) != Point:
		point2 = Point(*point2.flatten())
		
	# point1: Point, point2: Point
	return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)**0.5

def get_camera_pose():
	global arm
	return arm.get_current_pose(CAMERA_LINK_NAME).pose

def get_gripper_pose():
	global arm
	return arm.get_current_pose(GRIPPER_LINK_NAME).pose
	
def pose_from_vector3D(waypoint):
	#http://lolengine.net/blog/2013/09/18/beautiful-maths-quaternion-from-vectors
	pose= Pose()
	pose.position.x = waypoint[0]
	pose.position.y = waypoint[1]
	pose.position.z = waypoint[2] 
	#calculating the half-way vector.
	u = [1,0,0]
	norm = np.linalg.norm(waypoint[3:])
	v = np.asarray(waypoint[3:])/norm 
	if (u == v).all():
		pose.orientation.w = 1
		pose.orientation.x = 0
		pose.orientation.y = 0
		pose.orientation.z = 0
	elif (u == -v).all():
		pose.orientation.w = 0
		pose.orientation.x = 0
		pose.orientation.y = 0
		pose.orientation.z = 1
	else:
		half = [u[0]+v[0], u[1]+v[1], u[2]+v[2]]
		pose.orientation.w = np.dot(u, half)
		temp = np.cross(u, half)
		pose.orientation.x = temp[0]
		pose.orientation.y = temp[1]
		pose.orientation.z = temp[2]
	norm = np.sqrt(pose.orientation.x*pose.orientation.x + pose.orientation.y*pose.orientation.y + 
		pose.orientation.z*pose.orientation.z + pose.orientation.w*pose.orientation.w)
	if norm == 0:
		norm = 1
	pose.orientation.x /= norm
	pose.orientation.y /= norm
	pose.orientation.z /= norm
	pose.orientation.w /= norm
	return pose
	
def point_to_np(point):
	return np.array((point.x, point.y, point.z))
	
def move_gripper_to(destination):
	# print("Moving to position \n" + str(destination.position))
	
	# Set target and go
	global arm
	arm.set_start_state_to_current_state()
	arm.set_pose_target(destination, gripper)
	# trajectory = arm.plan()
	# arm.execute(trajectory)
	arm.go()
	rospy.sleep(1)
	
def pose_from_point(point, flower_location=None):
	# point, flower_location: Point
	if not flower_location:
		flower_location = Point(0.6,0,0.25)
	
	# Setup
	np_flower_location = np.asarray((flower_location.x, flower_location.y, flower_location.z))
	np_target_pose = np.asarray((point.x, point.y, point.z))
	
	# Compute orientation to face flower
	roll, pitch, yaw = TomCode.lookAt(np_target_pose, np_flower_location, [0, 1, 0])
	orientation_quaternion = TomCode.quaternion_from_euler(roll, pitch, yaw)
	
	# Construnct target pose
	target_pose = Pose()
	target_pose.position = point
	target_pose.orientation.x = -orientation_quaternion[0][0].item()
	target_pose.orientation.y = -orientation_quaternion[1][0].item()
	target_pose.orientation.z = orientation_quaternion[2][0].item()
	target_pose.orientation.w = orientation_quaternion[3][0].item()
	
	return target_pose
	
def locations_mean(locations):
	x = sum(location.x for location in locations) / len(locations)
	y = sum(location.y for location in locations) / len(locations)
	z = sum(location.z for location in locations) / len(locations)
	return Point(x,y,z)

def sample_upper_sphere(center, radius):
	# num_points: int, center: Point, radius: float
	# Uniformly samples the upper half-sphere above center
	# Using 3D Gaussian distribution due to its spherical symmetry (which causes normalized vector to be uniformly sampled over sphere)
	x_offset = np.random.normal()
	y_offset = np.random.normal()
	z_offset = abs(np.random.normal())  # Only view the flower from on top (hence the abs)
	offset_vector = np.array((x_offset, y_offset, z_offset))
	offset_vector = (offset_vector / np.linalg.norm(offset_vector)) * radius
	return Point(offset_vector[0] + center.x, offset_vector[1] + center.y, offset_vector[2] + center.z)

def sample_upper_sphere_within_reach(center, radius, robot_base):
	global arm
	sampled_location = None
	while radius > 0:
		for _ in range(MAX_SCAN_ATTEMPTS * 100):
			sampled_location = sample_upper_sphere(center, radius)
			
			# arm.set_start_state_to_current_state()
			# arm.set_pose_target(sampled_location, gripper)
			if get_distance(sampled_location, robot_base) <= ROBOT_REACH_RADIUS: # and arm.plan().joint_trajectory.points:
				return sampled_location
		radius -= RADIUS_STEP_SIZE  # if attempted distance is too far for robot to reach
		print("Robot only managed to reach point at radius of " + str(radius))
	exit("FAILED: robot can't reach flower or need more scan attempts")

def get_proximity_matrix(points):  # Upper triangular matrix because distances are undirected
	result = np.zeros((len(points), len(points)))
	for row in range(len(points)):
		for col in range(row + 1, len(points)):
			result[row, col] = get_distance(points[row], points[col])
	return result

def reorder_to_shortest_hamiltonian_path(points):
	current_position = get_gripper_pose().position
	proximity_mat = get_proximity_matrix([current_position] + points)
	indices = christofides.compute(proximity_mat)['Christofides_Solution']
	assert(indices[0] == 0)
	del(indices[0])
	del(indices[-1])
	return map(lambda idx: points[idx-1], indices)

def find_flower(distance_from_flower, flower_location, vid):
	# Sample NUM_IMGS_PER_SCAN images randomly from upper half-sphere around flower (of radius distance_from_flower) 
	# from which to photograph flower
	print("Scanning for flower at distance " + str(distance_from_flower))
	destinations = [sample_upper_sphere_within_reach(center=flower_location, radius=distance_from_flower, robot_base=Point(*ROBOT_BASE_LOCATION)) for _ in range(NUM_IMGS_PER_SCAN)]
	destinations = reorder_to_shortest_hamiltonian_path(destinations)
	destinations = [pose_from_point(point=location, flower_location=flower_location) for location in destinations]
	# print("Locations for the camera to photograph from: \n" + str(destinations) + "\n")
	# reorder_to_shortest_hamiltonian_path(destinations)
	
	# Navigate to those and photograph
	img_coords = []
	for destination in destinations:
		# print("Moving camera to \n" + str(destination) + "\n")
		move_gripper_to(destination)
		try:
			img_coords.append(TomCode.get_flower_im_coor(vid))
			print("Successfully photographed flower")
		except FlowerNotFoundError:
			print("couldn't find flower in photographed image")
			continue
		rospy.sleep(1)
	if len(img_coords) < MIN_SUFFICIENT_VIEWPOINTS:
		raise FlowerNotFoundError("failed to take enough photographs of the flower.")
	
	# Call triangulate_coords with images and sampled locations
	location_img_coord_pairs = list(zip(destinations, img_coords))
	flower_locations = []
	for pair1, pair2 in combinations(location_img_coord_pairs, 2):
		flower_location = TomCode.get3Dposition(pair1[1], pair2[1], pair1[0], pair2[0])
		flower_location = Point(flower_location[0,0], flower_location[0,1], flower_location[0,2])
		flower_locations.append(flower_location)
	# print("Estimated locations for the flower: \n" + str(flower_locations) + "\n")
	
	# Ensure locations found for the flower match up to within FLOWER_LOCATION_TOLERANCE, else raise FlowerNotFoundError
	num_neighbors = sum(get_distance(location1, location2) <= FLOWER_LOCATION_TOLERANCE for location1, location2 in combinations(flower_locations, 2))
	num_combinations = len(flower_locations) * (len(flower_locations) - 1) / 2  # Analytically computed len(combinations(location_img_coord_pairs, 2))
	if (num_combinations - num_neighbors) / num_combinations > MAX_OUTLIER_PERCENTAGE_TOLERANCE:
		print("Took " + str(len(location_img_coord_pairs)) + " successful pictures of flower, but " \
			  + str(num_combinations - num_neighbors) + " pairs of my estimations for its locations are too far apart.")
		print("Distances between estimations: " + str(["{:.3f}".format(get_distance(location1, location2)) for location1, location2 in combinations(flower_locations, 2)]))
		raise FlowerNotFoundError("failed to triangulate flower, probably due to too many misidentifications of flower.")
	
	return locations_mean(flower_locations)

def request_help():
	exit("FAILED")
	

# -----------------------------------Main control module-----------------------------------
def HW_part1():	
	vid = cv2.VideoCapture(0)
	
	flower_location = PLANT_LOCATION
	radius = get_distance(get_camera_pose().position, flower_location) * INITIAL_SCAN_DISTANCE_PROPORTION
	while get_distance(get_camera_pose().position, flower_location) >= ACCEPTABLE_PROXIMITY:
		for flower_scan_attempt_no in range(MAX_SCAN_ATTEMPTS):
			try:
				new_flower_location = find_flower(radius, flower_location, vid)
				if get_distance(new_flower_location, flower_location) > ACCEPTABLE_CORRECTION:
					raise FlowerNotFoundError("too large of a correction was predicted: \n" + new_flower_location + "\n")
					continue
				flower_location = new_flower_location
				print("Concluded that flower is located at \n" + str(flower_location) + "\nMoving closer.\n\n\n")
				radius -= RADIUS_STEP_SIZE
				break
			except FlowerNotFoundError as flower_detection_error:
				print("Attempt number " + str(flower_scan_attempt_no + 1) + " failed: flower not found because " + str(flower_detection_error) + "\n")
				continue
		if flower_scan_attempt_no == MAX_SCAN_ATTEMPTS-1:
			request_help()
	print("Got close enough.")

def main():
	print("Beginning assignment 1 solution")
	np.random.seed(0)
	init_moveit()
	HW_part1()
	destroy_moveit()
	print("Assignment 1 complete")
	
if __name__ == "__main__":
	main()
