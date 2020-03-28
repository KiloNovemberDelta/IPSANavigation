<h1>Requirement</h1>

Basic Installation. Go to your terminal (CTRL+ALT+T), and write the following command. You need a password the first time you use the command sudo (superuser do). 

* python3 : `sudo apt install -y python3 python3-dev build-essential` 

* git : `sudo apt install git`
* pip : `sudo apt-get install python3-pip python3-yaml`

Ubuntu 18.04 LTS or Ubuntu 16.04 LTS can be selected. Follow the right tutorial according to your OS

* **Ubuntu 18.04 LTS Bionic**

ROS Melodic : http://wiki.ros.org/melodic/Installation/Ubuntu

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-melodic-desktop-full
	
sudo rosdep init

sudo rosdep update

echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```

Running test : 

```bash
source ~/.bashrc
roscore & rosrun gazebo_ros gazebo
```



* **Ubuntu 16.04 LTS Xenial**

ROS Kinetic : http://wiki.ros.org/kinetic/Installation/Ubuntu

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

sudo apt update

sudo apt install ros-kinetic-desktop-full
	
sudo rosdep init

sudo rosdep update

echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
```

Running test :

```bash
source ~/.bashrc
roscore & rosrun gazebo_ros gazebo
```

* **BOTH Ubuntu 18.04 LTS Bionic & Ubuntu 16.04 LTS Xenial**

Install rospy for Python3 

```bash
sudo pip3 install rospkg
```




<h1>Run the code </h1>

We create a folder "GAZEBO" within the "Documents" folder. There, we copy the code. 

```shell
cd ~/Documents/
mkdir GAZEBO
cd GAZEBO
git clone https://github.com/KiloNovemberDelta/Navigation
```

Then we build and source the code

```bash
cd ~/Documents/GAZEBO/
catkin_make install
source devel/setup.bash
```

Finally, we run the environment 

```
roslaunch rover_ipsa rover_ipsa_world.launch
```
In another terminal, run the behavior of the robot (in the "Controller" folder)

```
python3 main.py
```



![Simulation Environment](https://github.com/KiloNovemberDelta/Navigation/blob/master/pic.png)