
# Com action
'''
roslaunch talos_gazebo talos_gazebo.launch

roslaunch talos_pal_locomotion talos_dcm_walking_controller.launch estimator:=kinematic_estimator_params

rosrun talos_wbc_actions push_com_action -d 3.0 -x 0.0 -y 0.1 -z 0.0
'''