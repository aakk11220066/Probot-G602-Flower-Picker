#! source ~/probot_g602_ws/devel/setup.bash
#! python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from moveit_commander import MoveGroupCommander
from copy import deepcopy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from probot_msgs.msg import ControllerCtrl
import math
# define a video capture object


def callback(data):
    if (data.ctrl == 6):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


class ProbotDemo:

    def get_object_im_coor(self):
        xp=-1
        yp=-1
        # ret, frame = self.vid.read()
        # cv2.imshow('frame', frame)
        # cv2.waitKey(50)
        detected = False
        # while (detected is not True):
        for i in range(5):
            # Take each frame
            _, img = self.vid.read()
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
                #                     1, (255, 0, 0), 2, cv2.LINE_AA)
            # cv2.drawContours(img, contours, -1, (0,255,0), 3)
            cv2.imshow('img', img)


            k = cv2.waitKey(5) & 0xFF
            if k == 27:
                break
        if xp<0:
            print('not detected')
            #raise FlowerNotFoundError()
        else:
            print('Detected:',(xp,yp))
        return xp,yp

    import math

    def euler_from_quaternion(self,x, y, z, w):
        print('orientation: ',x,y,z,w)
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
        print('R: ',R)
        return  R # in radians
    def getCameraPos(self):
        #camera pos = end-effector pos*Rot(=I)+Trans
        z_shift=0.1
        y_shift=0.02
        x_shift=0
        print('final')
        print('position:',self.arm.get_current_pose().pose.position)
        print('orient:', self.arm.get_current_pose().pose.orientation)
        return

    def get3Dposition(self,im1P,im2P,f1P,f2P):
        # projection matrix
        # 1
        rotation_mat = np.zeros(shape=(3, 3))
        Rq1=self.euler_from_quaternion(f1P.orientation.x,f1P.orientation.y,f1P.orientation.z,f1P.orientation.w)
        R1 = cv2.Rodrigues(Rq1, rotation_mat)[0]
        T1= np.asarray([f1P.position.x,f1P.position.y,f1P.position.z]).T
        print('R1:',R1)
        print('T1:', T1)
        PA1 = np.concatenate((np.dot(self.mtx, R1), np.dot(self.mtx,T1)), axis=1)
        # 2
        rotation_mat = np.zeros(shape=(3, 3))
        Rq2 = self.euler_from_quaternion(f2P.orientation.x, f2P.orientation.y, f2P.orientation.z, f2P.orientation.w)
        R2 = cv2.Rodrigues(Rq2, rotation_mat)[0]
        PA2 = np.concatenate((np.dot(self.mtx, R2), np.dot(self.mtx, np.asarray(f2P.position))), axis=1)
        points3D = cv2.triangulatePoints(PA1, PA2, im1P, im2P)
        points3D /= points3D[3]
        points3d = points3D.T[:, :3]
        print('points3d\n', points3d)
        return points3d
    def __init__(self):
        cam1_points = []
        cam2_points = []
        self.vid = cv2.VideoCapture(0)
        with np.load('/home/tom/probot_g602_ws/src/probot_g602/probot_g602_demo/scripts/Camera_params.npz') as X:
            self.mtx, dist, rvecs, tvecs = [X[i] for i in ('mtx', 'dist', 'rvecs', 'tvecs')]

        moveit_commander.roscpp_initialize(sys.argv)


        rospy.init_node('probot_demo')


        rospy.Subscriber("/probot_controller_ctrl", ControllerCtrl, callback)

        arm = moveit_commander.MoveGroupCommander('manipulator')

        end_effector_link = arm.get_end_effector_link()

        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)

        arm.allow_replanning(True)

        arm.set_goal_position_tolerance(0.9)
        arm.set_goal_orientation_tolerance(0.5)

        arm.set_max_acceleration_scaling_factor(0.15)
        arm.set_max_velocity_scaling_factor(0.2)

        arm.set_named_target('home')
        arm.go()

        #new
        rospy.loginfo("Hello ROS!")
        # ret, frame = vid.read()
        # cv2.imshow('frame', frame)
        # cv2.waitKey(50)
        x1,y1=self.get_object_im_coor()
        cam1_points.append((x1, y1))
        f1=arm.get_current_pose(end_effector_link).pose
        print(arm.get_current_joint_values())
        print(arm.get_current_pose())
        position1_up = [0.5230998396873474, -0.22761116921901703, -0.35051265358924866, -7.85741867730394e-05,
                        0.5781247615814209, 0.5230510234832764]
        position1_down = [0.5230904817581177, -0.6168404221534729, -0.4656371772289276, 9.407393372384831e-05,
                          1.0824624300003052, 0.5231050252914429]
        position2_up = [-0.5423908829689026, -0.44967007637023926, -0.004831122234463692, -1.0700880920921918e-05,
                        0.45458924770355225, -0.5425480008125305]
        position2_down = [-0.5425540208816528, -0.7636317610740662, -0.13111212849617004, 9.008259803522378e-05,
                          0.8949062824249268, -0.5426883697509766]

        arm.set_joint_value_target(position1_up)
        arm.go()
        arm.set_joint_value_target(position1_down)
        arm.go()
        #self.get_object_im_coor()
        x2,y2=self.get_object_im_coor()
        cam2_points.append((x2, y2))
        f2 = arm.get_current_pose(end_effector_link).pose
        self.get3Dposition(cam1_points,cam2_points,f1,f2)
        print(arm.get_current_joint_values())
        print(arm.get_current_pose())
        # ret, frame = vid.read()
        # cv2.imshow('frame', frame)
        # cv2.waitKey(50)

        arm.set_joint_value_target(position1_up)
        arm.go()
        self.get_object_im_coor()
        arm.set_joint_value_target(position2_up)
        arm.go()
        self.get_object_im_coor()
        arm.set_joint_value_target(position2_down)
        arm.go()
        self.get_object_im_coor()
        arm.set_joint_value_target(position2_up)
        arm.go()
        self.get_object_im_coor()
        print('final')
        print('position:',arm.get_current_pose().pose.position)
        print('orient:', arm.get_current_pose().pose.orientation)
        arm.set_named_target('home')
        arm.go()

        start_pose = arm.get_current_pose(end_effector_link).pose

        waypoints = []

        # waypoints.append(start_pose)

        wpose = deepcopy(start_pose)
        wpose.position.z -= 0.2
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.y += 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.x -= 0.1
        wpose.position.y -= 0.1
        waypoints.append(deepcopy(wpose))

        wpose.position.x += 0.1
        waypoints.append(deepcopy(wpose))

        fraction = 0.0
        maxtries = 100
        attempts = 0

        arm.set_start_state_to_current_state()

        while fraction < 1.0 and attempts < maxtries:
            (plan, fraction) = arm.compute_cartesian_path(
                waypoints,  # waypoint poses
                0.01,  # eef_step
                0.0,  # jump_threshold
                True)  # avoid_collisions

            attempts += 1

            if attempts % 10 == 0:
                rospy.loginfo("Still trying after " + str(attempts) + " attempts...")

        if fraction == 1.0:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo(
                "Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")

        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        target_position = [-0.012496701441705227, -0.41917115449905396, -0.19433578848838806, 0.018087295815348625,
                           2.2288901805877686, -0.013513846322894096]
        arm.set_joint_value_target(target_position)
        arm.go()

        target_position = [-0.012496701441705227, -0.2822195291519165, -0.05068935826420784, 0.030009545385837555,
                           1.9483672380447388, 2.1740968227386475]
        arm.set_joint_value_target(target_position)
        arm.go()

        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    print('hello')
    ProbotDemo()
