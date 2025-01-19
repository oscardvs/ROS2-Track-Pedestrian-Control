# ROS2-Track-Pedestrian-Control

## **Task Description**

In this assignment, we implement two ROS 2 packages:

1. **opencv_person_detector:** A node that uses OpenCV to detect pedestrians in images from a camera embarked on a driving mirte robot.
 
2. **control_barrel_world:** A node that controls a mirte robot to follow a track defined by cones and stop when a pedestrian is detected at a defined distance.

Both packages are split into `_main.cpp` and `_node.cpp`. The `_node.cpp` file contains the main logic of the package, while the `_main.cpp` file is only responsible for initializing and running the node.

## **Packages and Nodes**

### **1. opencv_person_detector**

- This node first subscribes to camera images from mirte and publish these images into rviz, as well as detected pedestrians. The pedestrians are detected using OpenCV HOG descriptor. Once a person is detected, a red bounding box is drawn arround that person.

### **2. control_barrel_world**

- Subscribes to obstacle detections and pedestrian detections, read and publishes velocities. When the cones are detected, it calculates the distance to the robot and its position relative to the robot's centerline. Depending on the cone's position, the robot drives forward or steers in the adequate direction. While in this loop, it also check for pedestrian and stops when the person's bounding box area is larger than 20000px.
## **Header Files**

- `opencv_person_detector_node.hpp` **and** `control_barrel_world_node.hpp` contain the class definitions and declarations for the nodes. These header files define the node structures; member variables and functions, such as the callback functions for image and detection processing. This allows us to reuse the code across the nodes more easily.

## **Launch Files**

- **`solution.launch.xml`** in `control_barrel_world` package.
  - Launches the necessary nodes for the simulation.
  
## **XML Files and CMakeLists**

- `package.xml` files contain meta-information about the package such as dependencies, versions and my details. It makes sure that all the dependencies are installed.

- `CMakeLists.txt` files define how the code is compiled and linked. It specifies which libraries needs to be used and configures the build process for the colcon build.
  
## **Building Instructions**

1. **Create a new workspace and make a source (src) directory inside of it**

2. **Inside** `/<your_workspace>/src` **clone:**
   ```bash
   git clone git@gitlab.ro47003.me.tudelft.nl:students-2425/ro47003_mirte_simulator.git
   git clone git@gitlab.ro47003.me.tudelft.nl:students-2425/lab4/group42.git
   ```
3. **Build the ros2 packages**
   ```bash
   cd ..
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

## **Running Instructions**

1. **Source ROS 2 Humble:**

   ```bash
   source /opt/ros/humble/setup.bash
   ```
   
2. **If not build yet, go to building instructions. Otherwise, skip to 3.**

3. **Launch the launch file**
   ```bash
   cd <your_workspace>
   source install/setup.bash
   ros2 launch control_barrel_world solution.launch.xml gui:=true
   ```

Setting `gui:=true` launches gazebo with a graphical user interface, remove that extension if you are running on a low RAM machine.
