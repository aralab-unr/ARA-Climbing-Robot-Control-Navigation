#REU_2022_Climbing_Robot

Documentation

Software
Our potential field uses point-to-point locomotion for a magnetic robot to traverse a ferromagnetic structure. The software created uses a queue to place points on the bridge and retrieve them. The project is being done to save the lives of bridge inspectors who must hang dangerously in the air.

The input for our software to work correctly is a 3D map in .dae or .stl format. The input will allow our Climbing Robot(CR) to initialize the bridge and allow the user to add the orientation on the bridge they would like inspected. 

To replicate and run our software, you must have ROS installed http://wiki.ros.org/ROS/Installation. Download the source files to the correct directory. Then start ROS with the command roscore in one terminal. Open another terminal and run the command rosrun rviz rviz. Next, open another terminal, get into the catkin_ws and use the following command to execute the program: rosrun climbing_robot pathmodel.py.








Hardware
