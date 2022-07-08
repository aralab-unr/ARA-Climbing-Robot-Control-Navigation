#REU_2022_Climbing_Robot

Documentation

Software
Our potential field uses point-to-point locomotion for a magnetic robot to traverse a ferromagnetic structure. The software created uses a queue to place points on the bridge and retrieve them. The project is being done to save the lives of bridge inspectors who must hang dangerously in the air.

The input for our software to work correctly is a 3D map in .dae or .stl format. The input will allow our Climbing Robot(CR) to initialize the bridge and allow the user to add the orientation on the bridge they would like inspected. 

To replicate and run our software, you must have ROS installed http://wiki.ros.org/ROS/Installation. Download the source files to the correct directory. Then start ROS with the command roscore in one terminal. Open another terminal and run the command rosrun rviz rviz. Next, open another terminal, get into the catkin_ws and use the following command to execute the program: rosrun climbing_robot pathmodel.py.



Hardware
The CR we are using has a bicycle-like frame which allows for a tighter turning radius. The CR used in the testing has been rebuilt with new parts except for one wheel. Brandon Moore engineered the new parts from a former design engineered by Son Nguyen. A new design is implemented due to technical issues like the wheel housing breaking. Another reason for a new design was the Panda Latte melted its pins, and a raspberry pie replaced the onboard computer. Due to the raspberry pie being slightly larger than the Panda Latte and the battery being transitioned into a power bank, the upper housing platform needed to be modified to accompany these new design features.  

The magnetic wheels needed to be switched out for smaller wheels at first because they burnt the motor out. Unfortunately, as the project progressed, the smaller wheels did not have enough magnetic force to handle the transfer from one surface to another surface. One solution for the wheels not climbing was to give them treads; this seemed to give them traction, which helped but did not solve the solution. The next option was to put the larger wheels back on the bot to allow a more potent magnetic force.

Warning neodymium magnets are strong and can hurt you if they are not respected. Magnetic wheels need to be handled with care and due diligence. While working on the robot and changing the wheels, standard metal tools may become the enemy and inflict pain. Caution should be used when handling these wheels. 


Mapping

