# pick_and_place
repo to showcase pick and place with moveit2 and perception pipeline

## To visualize the urdf in rviz with no ros2 control
``` ros2 launch pick_and_place_description visualize_robot.launch.py ```

## To visualize the urdf in rviz with ros2 control (mock hardware system)
``` ros2 launch pick_and_place_description start_control.launch.py ```

## To launch the model in gazebo
``` ros2 launch pick_and_place_description start_gazebo.launch.py ```

## To launch the model in ignition
``` ros2 launch pick_and_place_description start_ignition.launch.py ```

## To launch the model with test environment in gazebo
``` ros2 launch pick_and_place_simulation start_simulation.launch.py ```

## To launch moveit
``` ros2 launch pick_and_place_moveit_config start_moveit.launch.py ```

## To run pick and place demo without vision
``` ros2 run pick_and_place_test_nodes test_pick_and_place ```

## To run the object detection node
``` ros2 run pick_and_place_detection object_pose_detector ```
