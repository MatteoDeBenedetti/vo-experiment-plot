#!/bin/bash
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.odometry_in_world_pose >> odom_world.txt
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.diff_pose >> diff_pose.txt 
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.odometry_heading >> odometry_heading.txt
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.ground_truth_heading >> gt_heading.txt
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.ground_truth_pose >> gt_pose.txt
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.perc_error >> perc_error.txt
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.travelled_distance >> travelled_distance.txt

pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.odometry_heading >> odom_heading.txt  
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.odometry_pitch >> odom_pitch.txt  
pocolog viso2_evaluation_imu.0.log -s /viso2_evaluation_imu.odometry_roll >> odom_roll.txt  

