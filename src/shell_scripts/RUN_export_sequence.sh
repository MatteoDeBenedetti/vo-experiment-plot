#!/bin/bash    

pocolog vicon.0.log -s /vicon.pose_samples >> vicon.txt
pocolog vicon.0.log -s /vicon.heading_samples >> vicon_heading.txt
pocolog vicon.0.log -s /vicon.roll_samples >> vicon_roll.txt
pocolog vicon.0.log -s /vicon.pitch_samples >> vicon_pitch.txt
pocolog imu.0.log -s /imu_stim300.orientation_samples_out >> imu.txt
pocolog imu.0.log -s /imu_stim300.pitch_samples_out >> imu_pitch.txt
pocolog imu.0.log -s /imu_stim300.roll_samples_out >> imu_roll.txt
pocolog imu.0.log -s /imu_stim300.heading_samples_out >> imu_heading.txt

