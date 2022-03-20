# ros_simple_test
## ROS install guide for Ubuntu 21.10
Update the package index:
```
sudo apt-get update
```
Install ros-desktop-full-dev deb package:
```
sudo apt-get install ros-desktop-full-dev
```

It might take a while, but thats it!

## ROS workspace setup
Create a folder named "catkin_ws" (you can use whatever name you want, but if it's your first time settinp up the ros workspace, follow suggested naming pattern):
```
mkdir catkin_ws
```
Go into that directory and create another folder name "src"
```
mkdir src
```
Copy the content of this repository to the "src" folder.
Switch back to "catkin_ws" folder.
Build all the packages inside "src" folder with a single command:
```
catkin_make
```
Check the path to the "catkin_ws" folder by using
```
pwd
```
Now you need to edit .bashrc file that contains instructions that are executed each time you open a new terminal window. 
We need to source our freshly created ROS workspace. Lets open .bashrc first (editing it may require -su accsess, if thats the case - use sudo prefix)
```
nano ~/.bashrc
```
Scroll to the end of the file and add a new string
```
source [path_to_catkin_ws]/devel/setup.bash
```
Save and close the file.
Thats it, you are ready to go!

## General case workspace setup
In case you are setting up a generic ROS workspace and not this exact repository.
Go to the "src" folder, it should be empty. Run the following command:
```
catkin_init_workspace
```
Go one level up to the "catkin_ws" folder:
```
cd ..
```
Run
```
catkin_make
```
Done.

## Running the code
Alright, to run the basic client-server-client example from this repository you will need to open four terminal windows.
In the first one - launch ROSCORE:
```
rocore
```
In the second window - run the server node:
```
rosrun server_pkg server_node
```
In the third window run the sensor client node:
```
rosrun client_pkg sensor_node 
```
You will see that the sensor will start sending measurements to the server.
And in the last window run client node:
```
rosrun client_pkg client_node 
```
The client node will send one request to the server and receive any data the server sends back.
