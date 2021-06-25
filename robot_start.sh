#!/bin/bash

source /opt/ros/noetic/setup.bash
source /home/alice3/catkin_ws/devel/setup.bash

export SCIVERBOSE=1
export SCI_DISABLE_TK=1
export DISABLE_JAVA_DETECTION=1

export SCILAB_PATH=~/scilab-5.5.2/share/scilab
export SCI_PATH=~/scilab-5.5.2/bin
export SCI=~/scilab-5.5.2/share/scilab

export PATH=~/scilab-5.5.2/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=~/scilab-5.5.2/lib/scilab${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export LD_LIBRARY_PATH=~/scilab-5.5.2/lib/thirdparty/redist${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
export LD_LIBRARY_PATH=~/scilab-5.5.2/lib/thirdparty${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

roscore&
rospack profile&
sleep 10s
rosbag record -a -x "(.*)/compressed(.*)" -o /home/ubuntu/logs/robocup_data.bag&

roslaunch alice_webot_bridge alice_webot_bridge.launch server_ip:=$ROBOCUP_MIRROR_SERVER_IP port:=$(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR))&
sleep 5s

roslaunch alice_manager alice_manager_test.launch&
sleep 10s

rostopic pub /robotis/enable_ctrl_module std_msgs/String "data: 'walking_module'"&
sleep 1s

rostopic pub /heroehs/alice_foot_step_generator/dsp std_msgs/Float64 "data: 0.15"&
rostopic pub /heroehs/alice_foot_step_generator/foot_z_swap std_msgs/Float64 "data: 0.055"&
rostopic pub /heroehs/alice_foot_step_generator/body_z_swap std_msgs/Float64 "data: 0.02"&
rostopic pub /heroehs/alice_foot_step_generator/y_zmp_convergence std_msgs/Float64 "data: -0.015"&
sleep 1s

rostopic pub /robotis/enable_ctrl_module std_msgs/String "data: 'torso_module'"&
sleep 1s

#rosservice call /heroehs/online_walking/set_balance_param "updating_duration: 1
#balance_param: {cob_x_offset_m: -0.01, cob_y_offset_m: -0.01, hip_roll_swap_angle_rad: 0.0,
#  foot_roll_gyro_p_gain: 0.1, foot_roll_gyro_d_gain: 0.0, foot_pitch_gyro_p_gain: 0.1,
#  foot_pitch_gyro_d_gain: 0.0, foot_roll_angle_p_gain: 0.1, foot_roll_angle_d_gain: 0.0,
#  foot_pitch_angle_p_gain: 0.1, foot_pitch_angle_d_gain: 0.0, foot_x_force_p_gain: 0.009,
#  foot_x_force_d_gain: 0.0, foot_y_force_p_gain: 0.009, foot_y_force_d_gain: 0.0, foot_z_force_p_gain: 0.005,
#  foot_z_force_d_gain: 0.0, foot_roll_torque_p_gain: 0.0, foot_roll_torque_d_gain: 0.0,
#  foot_pitch_torque_p_gain: 0.0, foot_pitch_torque_d_gain: 0.0, roll_gyro_cut_off_frequency: 40.0,
#  pitch_gyro_cut_off_frequency: 40.0, roll_angle_cut_off_frequency: 40.0, pitch_angle_cut_off_frequency: 40.0,
#  foot_x_force_cut_off_frequency: 40.0, foot_y_force_cut_off_frequency: 40.0, foot_z_force_cut_off_frequency: 40.0,
#  foot_roll_torque_cut_off_frequency: 40.0, foot_pitch_torque_cut_off_frequency: 40.0}" &
#sleep 5s

if [ $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10002 -o $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10022 ];then
    cd /home/alice3/catkin_ws/src/alice3_vsc/alice_commander/config
    rm -rf robot.cfg

    mv robot2.cfg robot.cfg
fi

if [ $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10001 -o $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10021 ];then
    rm -rf robocup_tree.xml
    mv robocup_tree2.xml robocup_tree.xml   
    cd /home/alice3/catkin_ws/src/alice3_vsc/alice_commander/config
    rm -rf robocup_tree.xml
    mv robocup_tree2.xml robocup_tree.xml   
fi

if [ $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10001 -o $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10021 ];then
    roslaunch robot_localization_particlefilter robot_localization_kinematic_keeper.launch&
fi

if [ $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10002 -o $(cut -d: -f2 <(echo $ROBOCUP_SIMULATOR_ADDR)) -eq 10022 ];then
    roslaunch robot_localization_particlefilter robot_localization_kinematic_player.launch&
fi
sleep 1s

roslaunch alice_commander alice_commander.launch&

ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc&

roslaunch darknet_ros darknet_ros.launch&
sleep 2s

rosrun alice_vision alice_vision_node&
sleep 2s

sleep 30000s

#rostopic pub /heroehs/alice_foot_step_generator/walking_command_gui alice_footstep_msgs/FootStepCommand "{command: 'forward', step_num: 1023, step_time: 0.75, step_length: 0.1, side_step_length: 0.1, step_angle_rad: 0.05}"&
#sleep 100000s
#roslaunch alice_manager alice_manager_webots_test.launch

#rostopic pub /base_module/pose std_msgs/String "data: 'base_pose'"
#sleep 10s

#rostopic pub /robotis/enable_ctrl_module std_msgs/String "data: 'none'"
#sleep 5s

#rostopic pub /robotis/enable_ctrl_module std_msgs/String "data: 'walking_module'"
#sleep 5s

#rostopic pub /heroehs/alice_foot_step_generator/dsp std_msgs/Float64 "data: '0.15'"
#rostopic pub /heroehs/alice_foot_step_generator/foot_z_swap std_msgs/Float64 "data: '0.055'"
#rostopic pub /heroehs/alice_foot_step_generator/body_z_swap std_msgs/Float64 "data: '0.02'"
#rostopic pub /heroehs/alice_foot_step_generator/y_zmp_convergence std_msgs/Float64 "data: '-0.01'"
#sleep 5s

#rostopic pub /heroehs/alice_foot_step_generator/walking_command_gui alice_footstep_msgs/FootStepCommand "{command: 'forward', step_num: 100, step_time: 0.75, step_length: 0.18, side_step_length: 0.1, step_angle_rad: 0.05}"