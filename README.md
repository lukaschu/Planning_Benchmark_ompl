# Planning_Benchmark
Basic Benchmark for a 6DoF robot, using the a variety of omplś kinodynamic planners <br>
The simulation is originally taken from https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation and we use vsc tools to import all other repos. We tested everything on ROS2 Humble.

## Build Process

1. Prerequisities:
   First we install the necessary version of moveit2
   ```
   echo "deb [trusted=yes] https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/ ./" | sudo tee /etc/apt/sources.list.d/moveit_moveit2_packages.list
   echo "yaml https://raw.githubusercontent.com/moveit/moveit2_packages/jammy-humble/local.yaml humble" | sudo tee /etc/ros/rosdep/sources.list.d/1-moveit_moveit2_packages.list
   sudo apt update
   sudo apt upgrade
   ```
2. Define workspace, import necessary repos 
   ```
   mkdir benchmark_ws && cd benchmark_ws
   mkdir src && cd src 
   ```
   
   > **NOTE:** Feel free to change `benchmark_ws` to whatever absolute path you want.
   >
   
   Clone the repo
   ```
   git clone git@github.com:lukaschu/Planning_Benchmark_ompl.git
   ```
   
   Use vcs to import the necessary repos (Not tested yet)
   ```
   vcs import src --input src/Planning_Benchmark_ompl/Planning_Benchmark.repos 
   ```
   Install all rosdependencies
   ```
   cd ..
   rosdep install --ignore-src --from-paths src -y
   ```
   Now the workspace should be complete

3. Build the workspace:
   ```
   colcon build
   ```
4. Run simulation with moveit geometric planner
   ```
   ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur10e
   ```
   In rviz you can now choose a desired pose and apply the plan and execute button in the bottom left
5. Run simulation with ompl planner
   ```
   ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py ur_type:=ur10e
   ```
   Now run the ompl planner
   ```
   ros2 launch benchmark_planning simulation_launch.py
   ```
   > **NOTE:** You can combine point 4. and 5. Meaning that you can first plan using moveit geometric planners
   > using the provided rviz interface, and then our planner.
   >
   
   




