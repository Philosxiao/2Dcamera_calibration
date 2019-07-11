#! /usr/bin/env python

# Copyright (c) 2016-2018, Rethink Robotics Inc.
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
from __future__ import print_function
import rospy
import argparse
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)

from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb

import math
import numpy as np
import json
from pykeyboard import PyKeyboard
from transforms3d import quaternions
import sys

import intera_interface
import intera_external_devices
from intera_interface import CHECK_VERSION
#######################################
offset_in_end=np.array([0,0,0.185])#length safty before pick                
down_distance=np.array([0,0,0.065])#pick down distance
offset_probe=np.array([0,0,0.065])#the length of probe, add it when standardize
#######################################



def only_grip(limb,c):
    # initialize interfaces
    print("Getting robot state...")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state()
    gripper = None
    original_deadzone = None
    def clean_shutdown():
        if gripper and original_deadzone:
            gripper.set_dead_zone(original_deadzone)
        print("Exiting example.")
    try:
        gripper = intera_interface.Gripper(limb + '_gripper')
    except (ValueError, OSError) as e:
        rospy.logerr("Could not detect an electric gripper attached to the robot.")
        clean_shutdown()
        return
    rospy.on_shutdown(clean_shutdown)

    def offset_position(offset_pos):
        cmd_pos = max(min(gripper.get_position() + offset_pos, gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))

    def update_velocity(offset_vel):
        cmd_speed = max(min(gripper.get_cmd_velocity() + offset_vel, gripper.MAX_VELOCITY), gripper.MIN_VELOCITY)
        gripper.set_cmd_velocity(cmd_speed)
        print("commanded velocity set to {0} m/s".format(cmd_speed))

    original_deadzone = gripper.get_dead_zone()
    # WARNING: setting the deadzone below this can cause oscillations in
    # the gripper position. However, setting the deadzone to this
    # value is required to achieve the incremental commands in this example
    gripper.set_dead_zone(0.001)
    rospy.loginfo("Gripper deadzone set to {}".format(gripper.get_dead_zone()))
    num_steps = 8.0
    percent_delta = 1.0 / num_steps
    velocity_increment = (gripper.MAX_VELOCITY - gripper.MIN_VELOCITY) * percent_delta
    position_increment = (gripper.MAX_POSITION - gripper.MIN_POSITION) * percent_delta
    bindings = {
    #   key: (function, args, description)
        'r': (gripper.reboot, [], "reboot"),
        'c': (gripper.calibrate, [], "calibrate"),
        'q': (gripper.close, [], "close"),
        'o': (gripper.open, [], "open"),
        '+': (update_velocity, [velocity_increment], "increase velocity by {0}%".format(percent_delta*100)),
        '-': (update_velocity, [-velocity_increment],"decrease velocity by {0}%".format(percent_delta*100)),
        's': (gripper.stop, [], "stop"),
        'u': (offset_position, [-position_increment], "decrease position by {0}%".format(percent_delta*100)),
        'i': (offset_position, [position_increment], "increase position by {0}%".format(percent_delta*100)),
    }

    done = False
    rospy.loginfo("Enabling robot...")
    rs.enable()

    print("Controlling grippers. Press ? for help, Esc to quit.")

    if c:
        if c in ['\x1b', '\x03']:
            done = True
        elif c in bindings:
            cmd = bindings[c]
            print("command: {0}".format(cmd[2]))
            cmd[0](*cmd[1])
        else:
            print("key bindings: ")
            print("  Esc: Quit")
            print("  ?: Help")
            for key, val in sorted(bindings.items(),
                                    key=lambda x: x[1][2]):
                print("  %s: %s" % (key, val[2]))
        return
    # force shutdown call if caught by key handler



def map_keyboard(limb):
    # initialize interfaces
    print("Getting robot state...")
    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state()
    gripper = None
    original_deadzone = None
    def clean_shutdown():
        if gripper and original_deadzone:
            gripper.set_dead_zone(original_deadzone)
        print("Exiting example.")
    try:
        gripper = intera_interface.Gripper(limb + '_gripper')
    except (ValueError, OSError) as e:
        rospy.logerr("Could not detect an electric gripper attached to the robot.")
        clean_shutdown()
        return
    rospy.on_shutdown(clean_shutdown)

    def offset_position(offset_pos):
        cmd_pos = max(min(gripper.get_position() + offset_pos, gripper.MAX_POSITION), gripper.MIN_POSITION)
        gripper.set_position(cmd_pos)
        print("commanded position set to {0} m".format(cmd_pos))

    def update_velocity(offset_vel):
        cmd_speed = max(min(gripper.get_cmd_velocity() + offset_vel, gripper.MAX_VELOCITY), gripper.MIN_VELOCITY)
        gripper.set_cmd_velocity(cmd_speed)
        print("commanded velocity set to {0} m/s".format(cmd_speed))

    original_deadzone = gripper.get_dead_zone()
    # WARNING: setting the deadzone below this can cause oscillations in
    # the gripper position. However, setting the deadzone to this
    # value is required to achieve the incremental commands in this example
    gripper.set_dead_zone(0.001)
    rospy.loginfo("Gripper deadzone set to {}".format(gripper.get_dead_zone()))
    num_steps = 8.0
    percent_delta = 1.0 / num_steps
    velocity_increment = (gripper.MAX_VELOCITY - gripper.MIN_VELOCITY) * percent_delta
    position_increment = (gripper.MAX_POSITION - gripper.MIN_POSITION) * percent_delta
    bindings = {
    #   key: (function, args, description)
        'r': (gripper.reboot, [], "reboot"),
        'c': (gripper.calibrate, [], "calibrate"),
        'q': (gripper.close, [], "close"),
        'o': (gripper.open, [], "open"),
        '+': (update_velocity, [velocity_increment], "increase velocity by {0}%".format(percent_delta*100)),
        '-': (update_velocity, [-velocity_increment],"decrease velocity by {0}%".format(percent_delta*100)),
        's': (gripper.stop, [], "stop"),
        'u': (offset_position, [-position_increment], "decrease position by {0}%".format(percent_delta*100)),
        'i': (offset_position, [position_increment], "increase position by {0}%".format(percent_delta*100)),
    }

    done = False
    rospy.loginfo("Enabling robot...")
    rs.enable()
    print("Controlling grippers. Press ? for help, Esc to quit.")
    while not done and not rospy.is_shutdown():
        c = intera_external_devices.getch()
        if c:
            if c in ['\x1b', '\x03']:
                done = True
            elif c in bindings:
                cmd = bindings[c]
                print("command: {0}".format(cmd[2]))
                cmd[0](*cmd[1])
            else:
                print("key bindings: ")
                print("  Esc: Quit")
                print("  ?: Help")
                for key, val in sorted(bindings.items(),
                                       key=lambda x: x[1][2]):
                    print("  %s: %s" % (key, val[2]))
    # force shutdown call if caught by key handler

def get_position_now(limb):
    current_pose = limb.endpoint_pose()
    x = current_pose['position'].x
    y = current_pose['position'].y
    z = current_pose['position'].z 

    rx = current_pose['orientation'].x
    ry = current_pose['orientation'].y
    rz = current_pose['orientation'].z
    w = current_pose['orientation'].w        
    matrix = quaternions.quat2mat([w,rx,ry,rz])
    pos_origin=np.array([x,y,z])
    pos=np.dot(matrix,offset_probe)+pos_origin
    x=pos[0]
    y=pos[1]
    z=pos[2]
    print(x,y,z,end=' ')
    print()
    
    
    file="/home/philos/Desktop/output.txt"
    with open(file, 'a') as f:
        json.dump(x,f,ensure_ascii=True)
        f.write(" ")
        json.dump(y,f,ensure_ascii=True)
        f.write(" ")
        json.dump(z,f,ensure_ascii=True)
        f.write(" ")
        f.write("\n")


def go_to_the_point(target_point,args,limb,waypoint,traj,target_quaterniond=[0.0,1.0,0.0,0.0]):

    target_point_x=target_point[0]
    target_point_y=target_point[1]
    target_point_z=target_point[2]

    
    target_quaterniond_x = target_quaterniond[0]
    target_quaterniond_y = target_quaterniond[1]
    target_quaterniond_z = target_quaterniond[2]
    target_quaterniond_w = target_quaterniond[3]
                
    if args.joint_angles and len(args.joint_angles) != len(joint_names):
        rospy.logerr('len(joint_angles) does not match len(joint_names!)')
        return None
    if (args.position is None and args.orientation is None
        and args.relative_pose is None):
        if args.joint_angles:
            waypoint.set_joint_angles(args.joint_angles, args.tip_name, joint_names)
        else:
            rospy.loginfo("No Cartesian pose or joint angles given. Using default")
            waypoint.set_joint_angles(joint_angles=None, active_endpoint=args.tip_name)
    else:

        endpoint_state = limb.tip_state(args.tip_name)
        if endpoint_state is None:
            rospy.logerr('Endpoint state not found with tip name %s', args.tip_name)
            return None
        pose = endpoint_state.pose

        if args.relative_pose is not None:
            if len(args.relative_pose) != 6:
                rospy.logerr('Relative pose needs to have 6 elements (x,y,z,roll,pitch,yaw)')
                return None
    # create kdl frame from relative pose
            rot = PyKDL.Rotation.RPY(args.relative_pose[3],
                                        args.relative_pose[4],
                                        args.relative_pose[5])
            trans = PyKDL.Vector(args.relative_pose[0],
                                    args.relative_pose[1],
                                    args.relative_pose[2])
            f2 = PyKDL.Frame(rot, trans)
    # and convert the result back to a pose message
            if args.in_tip_frame:
        # end effector frame
                pose = posemath.toMsg(posemath.fromMsg(pose) * f2)
            else:
        # base frame
                pose = posemath.toMsg(f2 * posemath.fromMsg(pose))
        else:
            if args.position is not None and len(args.position) == 3:
                pose.position.x = target_point_x
                pose.position.y = target_point_y
                pose.position.z = target_point_z
            #if args.orientation is not None and len(args.orientation) == 4:
                pose.orientation.x = target_quaterniond_x
                pose.orientation.y = target_quaterniond_y
                pose.orientation.z = target_quaterniond_z
                pose.orientation.w = target_quaterniond_w                                        
        poseStamped = PoseStamped()

        matrix = quaternions.quat2mat([pose.orientation.w,pose.orientation.x,pose.orientation.y,pose.orientation.z])
        pos_origin=np.array([pose.position.x,pose.position.y,pose.position.z])

        hand_pos=pos_origin-np.dot(matrix,offset_in_end)
        print(hand_pos)
        pose.position.x=hand_pos[0]
        pose.position.y=hand_pos[1]
        pose.position.z=hand_pos[2]

        poseStamped.pose = pose
        waypoint.set_cartesian_pose(poseStamped, args.tip_name, args.joint_angles)
                
    rospy.loginfo('Sending waypoint: \n%s', waypoint.to_string())

    traj.append_waypoint(waypoint.to_msg())

    result = traj.send_trajectory(timeout=args.timeout)
    return result

def main():
    """
    Move the robot arm to the specified configuration.
    Call using:
    $ rosrun intera_examples go_to_cartesian_pose.py  [arguments: see below]

    -p 0.4 -0.3 0.18 -o 0.0 1.0 0.0 0.0 -t right_hand
    --> Go to position: x=0.4, y=-0.3, z=0.18 meters
    --> with quaternion orientation (0, 1, 0, 0) and tip name right_hand
    --> The current position or orientation will be used if only one is provided.

    -q 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0
    --> Go to joint angles: 0.0 -0.9 0.0 1.8 0.0 -0.9 0.0 using default settings
    --> If a Cartesian pose is not provided, Forward kinematics will be used
    --> If a Cartesian pose is provided, the joint angles will be used to bias the nullspace

    -R 0.01 0.02 0.03 0.1 0.2 0.3 -T
    -> Jog arm with Relative Pose (in tip frame)
    -> x=0.01, y=0.02, z=0.03 meters, roll=0.1, pitch=0.2, yaw=0.3 radians
    -> The fixed position and orientation paramters will be ignored if provided

    """

    epilog = """
See help inside the example with the '?' key for key bindings.
    """

    rp = intera_interface.RobotParams()
    valid_limbs = rp.get_limb_names()
    if not valid_limbs:
        rp.log_message(("Cannot detect any limb parameters on this robot. "
                    "Exiting."), "ERROR")
        return


    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument(
        "-r", "--record_point_pose", type=bool,default=0,
        nargs='+',
        help="record pose or not")
    parser.add_argument(
        "-p", "--position", type=float,
        nargs='+',
        help="Desired end position: X, Y, Z")
    parser.add_argument(
        "-o", "--orientation", type=float,
        nargs='+',
        help="Orientation as a quaternion (x, y, z, w)")
    parser.add_argument(
        "-R", "--relative_pose", type=float,
        nargs='+',
        help="Jog pose by a relative amount in the base frame: X, Y, Z, roll, pitch, yaw")
    parser.add_argument(
        "-T", "--in_tip_frame", action='store_true',
        help="For relative jogs, job in tip frame (default is base frame)")
    parser.add_argument(
        "-q", "--joint_angles", type=float,
        nargs='+', default=[],
        help="A list of joint angles, one for each of the 7 joints, J0...J6")
    parser.add_argument(
        "-t",  "--tip_name", default='right_hand',
        help="The tip name used by the Cartesian pose")
    parser.add_argument(
        "--linear_speed", type=float, default=0.6,
        help="The max linear speed of the endpoint (m/s)")
    parser.add_argument(
        "--linear_accel", type=float, default=0.6,
        help="The max linear acceleration of the endpoint (m/s/s)")
    parser.add_argument(
        "--rotational_speed", type=float, default=1.57,
        help="The max rotational speed of the endpoint (rad/s)")
    parser.add_argument(
        "--rotational_accel", type=float, default=1.57,
        help="The max rotational acceleration of the endpoint (rad/s/s)")
    parser.add_argument(
        "--timeout", type=float, default=None,
        help="Max time in seconds to complete motion goal before returning. None is interpreted as an infinite timeout.")
    parser.add_argument(
        "-l", "--limb", dest="limb", default=valid_limbs[0],
        choices=valid_limbs,
        help="Limb on which to run the gripper keyboard example"
    )
    args = parser.parse_args(rospy.myargv()[1:])


    try:
        rospy.init_node('go_to_cartesian_pose_py')
        limb = Limb()

        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)
        
        wpt_opts = MotionWaypointOptions(max_linear_speed=args.linear_speed,
                                         max_linear_accel=args.linear_accel,
                                         max_rotational_speed=args.rotational_speed,
                                         max_rotational_accel=args.rotational_accel,
                                         max_joint_speed_ratio=1.0)
        
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_names = limb.joint_names()
        
        rate = rospy.Rate(10) # 10hz


        if args.record_point_pose :
            origin_trans=get_position_now(limb)
            return None
        
        while not rospy.is_shutdown():

            ch=raw_input("waiting for next step:")

            if ch=='r':
                origin_trans=get_position_now(limb)
            if ch=='g':
                map_keyboard(args.limb)
            if ch=='q':
                return None
            if ch=='d':
                only_grip(args.limb,'q')
                coordinates_text=raw_input("type in coordinates:")

                target_point=np.array([float(coordinates_text.split()[0]),float(coordinates_text.split()[1]),float(coordinates_text.split()[2])])
                
                traj_1 = MotionTrajectory(trajectory_options = traj_options, limb = limb)
                waypoint_1 = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

                result_up=go_to_the_point(target_point,args=args,limb=limb,waypoint=waypoint_1,traj=traj_1)
                only_grip(args.limb,'o')

                matrix = quaternions.quat2mat([args.orientation[3],args.orientation[0],args.orientation[1],args.orientation[2]])
                down_point=target_point+np.dot(matrix,down_distance)
                

                traj_2 = MotionTrajectory(trajectory_options = traj_options, limb = limb)
                waypoint_2 = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)
                result_down=go_to_the_point(down_point,args=args,limb=limb,waypoint=waypoint_2,traj=traj_2)

                only_grip(args.limb,'q')

                traj_3 = MotionTrajectory(trajectory_options = traj_options, limb = limb)
                waypoint_3 = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

                result_up=go_to_the_point(target_point,args=args,limb=limb,waypoint=waypoint_3,traj=traj_3)


                traj_4 = MotionTrajectory(trajectory_options = traj_options, limb = limb)
                waypoint_4 = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

                bin_point=np.array([0.35216134766,0.621893054464,0.371810527511])

                result_up=go_to_the_point(bin_point,args,limb,waypoint_4,traj_4)
                only_grip(args.limb,'o')


            if ch=='s':
                coordinates_text=raw_input("type in coordinates and quaterniond:")
                print(coordinates_text.split())
                target_point=[float(coordinates_text.split()[0]),float(coordinates_text.split()[1]),float(coordinates_text.split()[2])]
                target_quaterniond=[float(coordinates_text.split()[3]),float(coordinates_text.split()[4]),float(coordinates_text.split()[5]),float(coordinates_text.split()[6])]

                traj_new = MotionTrajectory(trajectory_options = traj_options, limb = limb)
                waypoint_new = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

                result=go_to_the_point(target_point,args=args,limb=limb,waypoint=waypoint_new,traj=traj_new,target_quaterniond=target_quaterniond)
                if result is None:
                    rospy.logerr('Trajectory FAILED to send')
                    return

                if result.result:
                    rospy.loginfo('Motion controller successfully finished the trajectory!')
                else:
                    rospy.logerr('Motion controller failed to complete the trajectory with error %s',result.errorId)

    except rospy.ROSInterruptException:
        rospy.logerr('Keyboard interrupt detected from the user. Exiting before trajectory completion.')


if __name__ == '__main__':
    main()
