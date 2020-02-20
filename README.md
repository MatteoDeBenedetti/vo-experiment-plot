# vo-experiment-plot
Matlab scripts to plot the results of a VO experiment

# RUNNING A VO EXPERIMENT:
This instructionsserve as a guide for the process of running a VO experiment, logging the data and then plotting the results.

### PREPARATION:
These are initial steps to be only performed once.

##### Switching on ExoTeR:
- set power supply to 34V and 5A
- flick switch on the back of the rover

##### Switching on Vicon:
- switch on the *gigantet* machine then the *ultranet* machine and launch vicon on the beast (follow the order 1-2-pc reported on the labels on the two machines)

### RECORD TEST SEQUENCE
Steps to record a test sequence (camera feed, IMU, vicon) to then run the VO on.

##### Set up the Motion Generator Task:
This task will execute the desired motion described in its config file located in:
```sh
~/rock/bundles/rover/config/orogen/
```
For more informations on how to set up the motion, refer to the motion generator task readme in https://github.com/esa-prl/control-orogen-motion_generator.

##### Run the sequence:
Use the following commands to execute the motion and record the necessary logs
```sh
cd ~/rock/bundles/exoter/scripts
./vo_test_gen.rb
```
at the end press ENTER to stop the script  
The script will stop and the log folder is now ready. It is called "yyyymmdd-hhmm" and it's located at "~/rock/bundles/exoter/logs".  
This folder will be referred to from now on as the "*sequence folder*".

##### Run the VO:
First, you need to point the VO to the just acquired logs.  
Open the VO script.

```sh
cd ~/rock/bundles/exoter/scripts
vim virtual_spartan.rb
```
change the reference to the *sequence folder*, for example here it points to 20200213-1741
```sh
15 # open log file 
16 log = Orocos::Log::Replay.open('~/rock/bundles/exoter/logs/20200213-1741/loccam.0.log',
17                                '~/rock/bundles/exoter/logs/20200213-1741/imu.0.log',
18                                '~/rock/bundles/exoter/logs/20200213-1741/control.0.log',
19                                '~/rock/bundles/exoter/logs/20200213-1741/vicon.0.log')
```
save it and exit the text editor.  

Now run the VO script
```sh
cd ~/rock/bundles/exoter/scripts
./virtual_spartan.rb
```
and wait for it to finish.  
The execution of this script will generate another log folder, it is called “yyyymmdd-hhmm” and it’s located at “~/rock/bundles/exoter/logs”.  
This folder will be referred to from now on as the "*experiment folder*".  

##### Export the VO logs:
The logs are generated in files that can only be read using the rock-replay command.  
To export them in txt move in their folders and run a dedicated shell script.

The shell scripts are available in this repo at "src/shell_scripts".  
Download them in a path of your choice 

For the sequence, move to the *sequence folder*:
```sh
cd ~/rock/bundles/exoter/logs/<sequence folder>
/<shell scripts path>/RUN_export_sequence.sh
```
For the esperiment, move to the *experiment folder*:
```sh
cd ~/rock/bundles/exoter/logs/<experiment folder>
/<shell scripts path>/RUN_export_sequence.sh
```

These 2 commands will generate the necessary txt files in the two respective folders.

##### Plot the results:
Copy the *sequence folder* and *experiment folder* in the log folder in the matlab workspace at "vo-experiment-plot/logs"

Open the script "src/plot_vo.m"  
Insert the sequence and experiment folder names at the beginning of the matlab script as follows:
```sh
path_ca = {
 '../logs/<experiment folder>'};
vicon_path_ca = {
 '../logs/<sequence folder>'};
legend_names_verbose = {
 'name for the test'};
 ```
To compare the results from multiple experiments, each generated from its own sequence, it's just necessary to add more elelents to each cell array (making sure each experiment corresponds to its sequence), as follows:
```sh
path_ca = {
 '../logs/<experiment folder 1>',
 '../logs/<experiment folder 2>'};
vicon_path_ca = {
 '../logs/<sequence folder 1>',
 '../logs/<sequence folder 2>'};
legend_names_verbose = {
 '<name for test 1>',
 '<name for test 2>'};
 ```
And run it.  
The script will generate the following plots:
- figure 1: VO estimation and ground truth on the XY plane
- figure 2: VO estimation and ground truth on the Z axis over time
- figure 3: VO position error on the x,y,z components and xyz error norm, all over distnace travelled
- figure 4: VO orientation error on the heading, pitch and roll components and rpy error norm, all over distnace travelled
 
 For example, in case there were 2 experiments:  
 - experiment folder 1: "20200107-1421"; generated from the sequence folder 1: "20200107-1408"; that you want to call: "navcam"
 - experiment folder 2: "20200107-1425"; generated from the sequence folder 2: "20200107-1414"; that you want to call: "loccam"

This would be the code to use:
```sh
path_ca = {
 '../logs/20200107-1421',
 '../logs/20200107-1425'};
vicon_path_ca = {
 '../logs/20200107-1408',
 '../logs/20200107-1414'};
legend_names_verbose = {
 'navcam',
 'loccam'};
 ```






