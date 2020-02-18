%% PLOT VO

% This is the script to plot the results of a VO experiment 
% (appropriately run, logged and organized as described in the readme)

% Requirements:
% - it was developed and tested in Matlab R2019b but should not have problems
% with olders versions
% - the sequence and experiment folders in ../logs (more details on them in the readme)
% - the folder \functions


%% KNOWN BUGS
% RANDOM ERROR 1: for reasons beyond magic, sometimes odom_world.txt contains an
% extra header line. either delete it manually, or change the headerlines
% param in textread from 2 to 3
% In general it is a good idea to quickly check that all the *.txt files have
% only 2 header lines, otherwise errors of dimentions mismatch will occur

% RANDOM ERROR 2: another fun thing that very rarely but indeed does happen is
% that in the middle of the vicon.txt file there will be random lines of text instead of data, like:
% pocolog.rb[INFO]: loading file info from ./vicon.0.idx...
% /vicon.pose_samples.time.microseconds /vicon.pose_samples.sourceFrame /vicon.pose_samples.targetFrame /vicon.pose_samples.position.data[0] /vicon.pose_samples.position.data[1] /vicon.pose_samples.position.data[2] /vicon.pose_samples.cov_position.data[0] /vicon.pose_samples.cov_position.data[1] /vicon.pose_samples.cov_position.data[2] /vicon.pose_samples.cov_position.data[3] /vicon.pose_samples.cov_position.data[4] /vicon.pose_samples.cov_position.data[5] /vicon.pose_samples.cov_position.data[6] /vicon.pose_samples.cov_position.data[7] /vicon.pose_samples.cov_position.data[8] /vicon.pose_samples.orientation.im[0] /vicon.pose_samples.orientation.im[1] /vicon.pose_samples.orientation.im[2] /vicon.pose_samples.orientation.re /vicon.pose_samples.cov_orientation.data[0] /vicon.pose_samples.cov_orientation.data[1] /vicon.pose_samples.cov_orientation.data[2] /vicon.pose_samples.cov_orientation.data[3] /vicon.pose_samples.cov_orientation.data[4] /vicon.pose_samples.cov_orientation.data[5] /vicon.pose_samples.cov_orientation.data[6] /vicon.pose_samples.cov_orientation.data[7] /vicon.pose_samples.cov_orientation.data[8] /vicon.pose_samples.velocity.data[0] /vicon.pose_samples.velocity.data[1] /vicon.pose_samples.velocity.data[2] /vicon.pose_samples.cov_velocity.data[0] /vicon.pose_samples.cov_velocity.data[1] /vicon.pose_samples.cov_velocity.data[2] /vicon.pose_samples.cov_velocity.data[3] /vicon.pose_samples.cov_velocity.data[4] /vicon.pose_samples.cov_velocity.data[5] /vicon.pose_samples.cov_velocity.data[6] /vicon.pose_samples.cov_velocity.data[7] /vicon.pose_samples.cov_velocity.data[8] /vicon.pose_samples.angular_velocity.data[0] /vicon.pose_samples.angular_velocity.data[1] /vicon.pose_samples.angular_velocity.data[2] /vicon.pose_samples.cov_angular_velocity.data[0] /vicon.pose_samples.cov_angular_velocity.data[1] /vicon.pose_samples.cov_angular_velocity.data[2] /vicon.pose_samples.cov_angular_velocity.data[3] /vicon.pose_samples.cov_angular_velocity.data[4] /vicon.pose_samples.cov_angular_velocity.data[5] /vicon.pose_samples.cov_angular_velocity.data[6] /vicon.pose_samples.cov_angular_velocity.data[7] /vicon.pose_samples.cov_angular_velocity.data[8]
% You will see an error like this:
% Trouble reading unsigned integer from file (row 6724, field 1) ==>
% Go to the line (6754 in this case) and manually delete the text rows

% RANDOM ERROR 3: sometimes the dimentions of vicon_pose (a cell array with the data from vicon.txt)
% and the heading, roll and pitch (from vicon_heading.txt etc)
% will not be the same, usually just by 1. change it manually cy copying the last value where needed


%%

clear all
close all
clc

addpath('functions')
addpath('../logs')


% CREATE THESE 3 CELL ARRAYS CONTAINING THE LOGS AND LEGEND NAMES
% path_ca = {
%     'logs/<experiment folder 1>',
%     'logs/<experiment folder 2>'};
% vicon_path_ca = {
%     'logs/<sequence folder 1>',
%     'logs/<sequence folder 2>'};
% legend_names_verbose = {
%     '<test name 1>',
%     '<test bane 2>'};


% EXPERIMENTS ALREADY INCLUDED IN LOGS

% % most recent test to recheck the whole VO experiment pipeline
% path_ca = {
%     '../logs/20200213-1646',
%     '../logs/20200213-1646'};
% vicon_path_ca = {
%     '../logs/20200213-1633',
%     '../logs/20200213-1633'};
% legend_names_verbose = {
%     'navcam',
%     'loccam'};

% % navcam best vs loccam L trajectory 
% path_ca = {
%     '../logs/20200107-1421',
%     '../logs/20200107-1425'};
% vicon_path_ca = {
%     '../logs/20200107-1408',
%     '../logs/20200107-1414'};
% legend_names_verbose = {
%     'navcam',
%     'loccam'};

% % autonav with spartan test
path_ca = {
    '../logs/autonav_L-trajectory'}; 
vicon_path_ca = {
    '../logs/autonav_L-trajectory'};
legend_names_verbose = {
    'autonav test'};


% OPTION FLAGS
% (they are mostly for backward compatibility with logs captured before some features were implemented.
% If the logs that are being plotted were acquired after the 13/02/2020 there should really be no need to change any of them)
CONTROL_FILE = false; control_path = ''; % old separated vicon path
IMU_FILE = false; imu_path = ''; % old separated imu path for imu debugging
ODOM_TIME = false; % set false if vicon and odom have no timestamp
VICON_FILE = true; % before gt from evaluation was used, now it's taken from the vicon log
VICON_FILE_RPY = false; % new vicon output with also hdg, roll, pitch
ODOM_FILE_RPY = false; % new odom file with also hdg, roll, pitch
LEGEND_NAMES_VERBOSE = true; % to use the new user-specified legend names instead of the old automatic test#


%%
n_logs = size(path_ca,1);

% Pre-allocations
unique_vicon_times = cell(1,n_logs);
dist_accum_ca = cell(1,n_logs);
xyz_norm = cell(1,n_logs);
rpy_norm = cell(1,n_logs);
diff_pose_ca = cell(1,n_logs);
odom_pose_ca = cell(1,n_logs);
gt_pose_ca = cell(1,n_logs);
vicon_pose_interp_ca = cell(1,n_logs);
if(CONTROL_FILE)
    control_ca = cell(1,n_logs);
end
if(IMU_FILE)
    imu_ca = cell(1,n_logs);
end
if(VICON_FILE)
    vicon_pose_ca = cell(1,n_logs);
end


for i=1:n_logs
    
    % Read odometry file
    [t, x, y, z, b, c, d, a]=textread(horzcat(path_ca{i},'/odom_world.txt'), ...
        '%f%f%f%f%*f%*f%*f%*f%*f%*f%*f%*f%*f%f%f%f%f%*[^\n]', 'headerlines', 3, 'delimiter', '\t');
    odom_pose = [t/10^6, x, y, z, a, b, c, d];
    
    % Read vicon file from old-style paths
    if(~VICON_FILE)
        vicon_path_ca{i} = path_ca{i};
        % read gt pose file
        [t, x, y, z, b, c, d, a]=textread(horzcat(path_ca{i},'/gt_pose.txt'), ...
            '%f%*f%*f%*f%*f%*f%*f%*f%f%f%f%*f%*f%*f%*f%*f%*f%*f%*f%*f%f%f%f%f%*[^\n]', 'headerlines', 2, 'delimiter', '\t');
        vicon_pose = [t/10^6, x, y, z, a, b, c, d];
    end
    
    % read control file (not used much anymore)
    if(CONTROL_FILE)
        % Read control file
        [tr, rot, hdg]=textread(horzcat(path_ca{i},'/control.txt'), ...
            '%f%f%f%*[^\n]', 'headerlines', 2, 'delimiter', '\t');
        
        % Read control time file
        t=textread(horzcat(path_ca{i},'/control_time.txt'), ...
            '%u', 'headerlines', 2);
        t(size(tr,1)) = t(end);
        control = [t/10^6, tr, rot];
    end
    
    % Read IMU file
    if(IMU_FILE)
        % read imu file
        [t, b, c, d, a]=textread(horzcat(path_ca{i},'/imu.txt'), ...
            '%f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%*f%f%f%f%f%*[^\n]', 'headerlines', 2, 'delimiter', '\t');
        imu = [t/10^6, a, b, c, d];
    end
    
    % Read Vicon pose
    if (VICON_FILE)
        % Read exoter pose file
        [t, x, y, z, b, c, d, a]=textread(horzcat(vicon_path_ca{i},'/vicon.txt'), ... 
            '%f%*f%*f%*f%*f%*f%*f%*f%f%f%f%*f%*f%*f%*f%*f%*f%*f%*f%*f%f%f%f%f%*[^\n]', 'headerlines', 2, 'delimiter', '\t');
        vicon_pose = [t/10^6, x, y, z, a, b, c, d];
    else
        vicon_path_ca{i} = path_ca{i};
        % read gt pose file
        [t, x, y, z, b, c, d, a]=textread(horzcat(path_ca{i},'/gt_pose.txt'), ...
            '%f%*f%*f%*f%*f%*f%*f%*f%f%f%f%*f%*f%*f%*f%*f%*f%*f%*f%*f%f%f%f%f%*[^\n]', 'headerlines', 2, 'delimiter', '\t');
        vicon_pose = [t/10^6, x, y, z, a, b, c, d];
    end
    
    % Read IMU file
    if(IMU_FILE)
        imu(:,6) = quaternion2heading(imu(:,2:5));
        imu(:,7) = quaternion2pitch(imu(:,2:5));
        imu(:,8) = quaternion2roll(imu(:,2:5));
    end
    
    % convert odom orientation QUAT to ZYX
    if (~ODOM_FILE_RPY)
        odom_pose(:,9) = quaternion2heading(odom_pose(:,5:8));
        odom_pose(:,10) = quaternion2pitch(odom_pose(:,5:8));
        odom_pose(:,11) = quaternion2roll(odom_pose(:,5:8));
    end
    
    % convert vicon orientation QUAT to ZYX
    if (~VICON_FILE_RPY)
        vicon_pose(:,9) = quaternion2heading(vicon_pose(:,5:8));
        vicon_pose(:,10) = quaternion2pitch(vicon_pose(:,5:8));
        vicon_pose(:,11) = quaternion2roll(vicon_pose(:,5:8));
    end
    
    % compute absolute start time
    start_times = [odom_pose(1,1)]; 
    if(CONTROL_FILE)
        start_times = [start_times control_pose(1,1)];
    end
    if(IMU_FILE)
        start_times = [start_times imu(1,1)];
    end
    if(VICON_FILE)
        start_times = [start_times vicon_pose(1,1)];
    end
    start_time = min(start_times);
    
    % Apply time alignment
    odom_pose(:,1) = (odom_pose(:,1) - start_time);
    if(CONTROL_FILE)
        control(:,1) = (control(:,1) - start_time);
    end
    if(IMU_FILE)
        imu(:,1) = (imu(:,1) - start_time);
    end
    if(VICON_FILE)
        vicon_pose(:,1) = (vicon_pose(:,1) - start_time);
    end
    
    % save in cell array and clear loop vars
    odom_pose_ca{i} = odom_pose; clear odom_pose;
    if(CONTROL_FILE)
        control_ca{i} = control; clear control;
    end
    if(IMU_FILE)
        imu_ca{i} = imu; clear imu;
    end
    if(VICON_FILE)
        vicon_pose_ca{i} = vicon_pose; clear vicon_pose;
    end
    clear start_times, clear start_time
    
    %interpolate gt_pose to fill all the idx of odo_pose - position
    [unique_vicon_times{i}, unique_idx] = unique(vicon_pose_ca{i}(:,1));
    vicon_pose_interp_ca{i}(:,2) = interp1(unique_vicon_times{i}, vicon_pose_ca{i}(unique_idx,2), odom_pose_ca{i}(:,1), 'pchip');
    vicon_pose_interp_ca{i}(:,3) = interp1(unique_vicon_times{i}, vicon_pose_ca{i}(unique_idx,3), odom_pose_ca{i}(:,1), 'pchip');
    vicon_pose_interp_ca{i}(:,4) = interp1(unique_vicon_times{i}, vicon_pose_ca{i}(unique_idx,4), odom_pose_ca{i}(:,1), 'pchip');
    
    %interpolate gt_pose to fill all the idx of odo_pose - orientation
    [unique_vicon_times{i}, unique_idx] = unique(vicon_pose_ca{i}(:,1));
    vicon_pose_interp_ca{i}(:,9) = interp1(unique_vicon_times{i}, vicon_pose_ca{i}(unique_idx,9), odom_pose_ca{i}(:,1), 'pchip');
    vicon_pose_interp_ca{i}(:,10) = interp1(unique_vicon_times{i}, vicon_pose_ca{i}(unique_idx,10), odom_pose_ca{i}(:,1), 'pchip');
    vicon_pose_interp_ca{i}(:,11) = interp1(unique_vicon_times{i}, vicon_pose_ca{i}(unique_idx,11), odom_pose_ca{i}(:,1), 'pchip');
    
    % distance travelled from start at each t step
    dist_accum_ca{i} = zeros(size(odom_pose_ca{i}(:,1),1),1);
    dist_accum_ca{i}(1) = norm(vicon_pose_interp_ca{i}(1,2:4) - vicon_pose_ca{i}(1,2:4));
    for j = 2:size(odom_pose_ca{i}(:,1),1)
        dist = norm(vicon_pose_interp_ca{i}(j,2:4) - vicon_pose_interp_ca{i}(j-1,2:4));
        dist_accum_ca{i}(j,1) = dist + dist_accum_ca{i}(j-1,1);
    end
    
    % compute xyz error norm
    for j = 1:size(odom_pose_ca{i}(:,1),1)
        xyz_norm{i}(j,1) = norm([vicon_pose_interp_ca{i}(j,2) - odom_pose_ca{i}(j,2), ...
            vicon_pose_interp_ca{i}(j,3) - odom_pose_ca{i}(j,3), ...
            vicon_pose_interp_ca{i}(j,4) - odom_pose_ca{i}(j,4)]);
    end
    
    % compute rpy error norm
    for j = 1:size(odom_pose_ca{i}(:,1),1)
        rpy_norm{i}(j,1) = norm([vicon_pose_interp_ca{i}(j,9) - odom_pose_ca{i}(j,9), ...
            vicon_pose_interp_ca{i}(j,10) - odom_pose_ca{i}(j,10), ...
            vicon_pose_interp_ca{i}(j,11) - odom_pose_ca{i}(j,11)]);
    end
    
end


%% Define legend names 

legend_names = cell(1,n_logs);
if(LEGEND_NAMES_VERBOSE)
    legend_names = legend_names_verbose;
else
    for i = 1:n_logs
        legend_names{i} = horzcat('test ', num2str(i));
    end
end

legend_names_and2percent = cell(1,n_logs);
if(LEGEND_NAMES_VERBOSE)
    legend_names_and2percent = legend_names_verbose;
    legend_names_and2percent{i+1} = '2% distance travelled';
else
    for i = 1:n_logs
        legend_names_and2percent{i} = legend_names{i};
    end
    legend_names_and2percent{i+1} = '2% distance travelled';
end

legend_names_and_gt = cell(1,2*n_logs);
if(LEGEND_NAMES_VERBOSE)
    j = 1;
    for i = 1:2:2*n_logs-1
        legend_names_and_gt{i} = legend_names_verbose{j};
        j = j + 1;
    end
    j = 1;
    for i = 2:2:2*n_logs
        legend_names_and_gt{i} = horzcat(legend_names_verbose{j}, ' gt');
        j = j + 1;
    end
else
    j = 1;
    for i = 1:2:2*n_logs-1
        legend_names_and_gt{i} = legend_names{j};
        j = j + 1;
    end
    j = 1;
    for i = 2:2:2*n_logs
        legend_names_and_gt{i} = horzcat(legend_names{j}, ' gt');
        j = j + 1;
    end
end


%% plot trajectory and gt

% % ground truth trajectory vs VO estimate on xy plane
figure(1), close(1), figure(1);
hold on
for i=1:n_logs
    plot(odom_pose_ca{i}(:,2), odom_pose_ca{i}(:,3), '-*');
    plot(vicon_pose_ca{i}(:,2), vicon_pose_ca{i}(:,3), 'green', 'LineWidth', 1)
    xlabel('x [m]'), ylabel('y [m]')
end
title({'Visual Odometry Evaluation', 'VO estimate and GT'});
grid on, %axis equal;
legend(legend_names_and_gt);
hold off

% % ground truth trajectory vs VO estimate on z axis over time
figure(2), close(2), figure(2);
hold on
for i=1:n_logs
    plot(odom_pose_ca{i}(:,1), odom_pose_ca{i}(:,4), '-*');
    plot(vicon_pose_ca{i}(:,1), vicon_pose_ca{i}(:,4), 'green', 'LineWidth', 1)
    ylabel('z [m]'), xlabel('t [s]')
end
title({'Visual Odometry Evaluation', 'VO estimate and GT'});
grid on, %axis equal;
legend(legend_names_and_gt);
hold off


%% plot position error

if (VICON_FILE)
    figure(3), close(3), figure(3);
    
    subplot(2,2,1)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), vicon_pose_interp_ca{i}(:,2) - odom_pose_ca{i}(:,2), '-*');
        grid on; ylabel('X error [m]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'x error component over distance'});
    end
    plot(dist_accum_ca{1}(:,1), 0.02*dist_accum_ca{1}(:,1), 'r--')
    legend(legend_names_and2percent);
    hold off;
    
    subplot(2,2,2)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), vicon_pose_interp_ca{i}(:,3) - odom_pose_ca{i}(:,3), '-*');
        grid on; ylabel('Y error [m]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'y error component over distance'});
    end
    plot(dist_accum_ca{1}(:,1), 0.02*dist_accum_ca{1}(:,1), 'r--')
    legend(legend_names_and2percent);
    hold off;
    
    subplot(2,2,3)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), vicon_pose_interp_ca{i}(:,4) - odom_pose_ca{i}(:,4), '-*');
        grid on; ylabel('Z error [m]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'z error component over distance'});
    end
    plot(dist_accum_ca{1}(:,1), 0.02*dist_accum_ca{1}(:,1), 'r--')
    legend(legend_names_and2percent);
    hold off;
    
    subplot(2,2,4)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), xyz_norm{i}(:,1), '-*');
        grid on; ylabel('XYZ error norm [m]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'xyz error norm over distance'});
    end
    plot(dist_accum_ca{1}(:,1), 0.02*dist_accum_ca{1}(:,1), 'r--')
    legend(legend_names_and2percent);
    hold off;
end


%% plot orientation error

% hdg pitch roll errors from vicon and odom interp and synchronized over dist travelled
if (VICON_FILE)
    figure(4), close(4), figure(4);
    
    subplot(2,2,1)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), rad2deg(vicon_pose_interp_ca{i}(:,9) - odom_pose_ca{i}(:,9)), '-*');
        grid on; ylabel('heading error [deg]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'heading error component over distance'});
    end
    legend(legend_names);
    hold off;
    
    subplot(2,2,2)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), rad2deg(vicon_pose_interp_ca{i}(:,10) - odom_pose_ca{i}(:,10)), '-*');
        grid on; ylabel('pitch error [deg]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'pitch error component over distance'});
    end
    legend(legend_names);
    hold off;
    
    subplot(2,2,3)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), rad2deg(vicon_pose_interp_ca{i}(:,11) - odom_pose_ca{i}(:,11)), '-*');
        grid on; ylabel('roll error [deg]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'roll error component over distance'});
    end
    legend(legend_names);
    hold off;
    
    subplot(2,2,4)
    hold on;
    for i=1:n_logs
        plot(dist_accum_ca{i}(:,1), rpy_norm{i}(:,1), '-*');
        grid on; ylabel('rpy error norm [deg]'), xlabel('Distance traveled [m]');
        title({'Visual Odometry Evaluation - ', 'rpy error norm over distance'});
    end
    legend(legend_names);
    hold off;
end




