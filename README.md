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
