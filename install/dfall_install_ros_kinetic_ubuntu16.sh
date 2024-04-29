#!/bin/bash

# --------------------------------------------------------------- #
# USAGE:
# 1) Download this installation shell script file
# 2) Modify the file permission to be executable
#    >> chmod +x dfall_install_ros_melodic_ubuntu16.sh
# 3) Execute the script
#    >> ./dfall_install_ros_melodic_ubuntu16.sh <agentID>
#    where <agentID> is replaced by a number, eg. 1



# --------------------------------------------------------------- #
# PROCESS THE INPUT ARGUMENTS

die () {
    echo >&2 "$@"
    exit 1
}

# Check the input argument is supplied and correct
[ "$#" -eq 1 ] || die "1 argument required (AgentID), $# provided"
echo $1 | grep -E -q '^[0-9]+$' || die "Numeric argument required, $1 provided"



# --------------------------------------------------------------- #
# INSTALL ROS AS PER:
# http://wiki.ros.org/melodic/Installation/Ubuntu

# Setup the computer to accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup the keys
#sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update the package index, i.e., the list of programs
sudo apt-get update

# Install all available upgrades
# > Note: the -y option means Automatic yes to prompts
sudo apt-get -y upgrade

# Install ROS Kinetic
# > Note: we use the configuration: "Desktop-Full Install: (Recommended)"
sudo apt-get -y install ros-kinetic-desktop-full

# > Note: for installing on a "headless" operating system, i.e., a Raspberry Pi
#   without a graphical desktop, use the configuration: "ROS-Base: (Bare Bones)"
#sudo apt -y install ros-kinetic-ros-base

# Initialise and update rosdep
# > Note: rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS
sudo rosdep init
rosdep update

# Add the ROS environment setup to the .bashrc
# NOTE: this is added to the .bashrc at the end to keep all such
#       additions in one place
#echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
# Run it now so that it is valid for this terminal session
source /opt/ros/kinetic/setup.bash



# --------------------------------------------------------------- #
# INSTALL THE PYTHON USB PACKAGE
# > Note: this is needed to connected to the Crazyradio USB dongle

# Install the python package management system "python-pip"
sudo apt-get -y install python-pip

# Install the python USB package "pyusb"
sudo pip install pyusb



# --------------------------------------------------------------- #
# INSTALL THE NECESSARY Qt5 PACKAGES AND CREATOR
# > NOTE: Qt is required for the GUIs, and the dfall ROS package
#         requires Qt for compilation

# Install the default Qt5
sudo apt -y install qt5-default

# Install the development version of the Qt5svg library
sudo apt -y install libqt5svg5-dev

# Install the charting library
sudo apt install libqt5charts5-dev

# Install the multimedia library (used for sound effects)
sudo apt install libqt5multimedia5
sudo apt install qtmultimedia5-dev

# Install the "Qt Creator" IDE
# > NOTE: this is not necessary to compile and run the dfall ROS package,
#         but it is required to edit the GUIs
sudo apt -y install qtcreator



# --------------------------------------------------------------- #
# CLONE THE dfall-system REPOSITORY

# Make the "dfall" directory under the users root
# > Note: the -p option means: no error if existing, make parent directories as needed
mkdir -p ~/dfall

# Change directory to this folder
cd ~/dfall

# Install git
sudo apt -y install git

# Clone the dfall-system git repository
git clone https://gitlab.ethz.ch/dfall/dfall-system.git



# --------------------------------------------------------------- #
# CREATE, COPY, EDIT A FEW NECESSARY FILES
# > NOTE: the "man sh" explain that the -c option specifies "sh" to:
#         "Read commands from the command_string operand instead of from the standard input.

# Add the dfallmaster's IP address to the /etc/hosts file
sudo sh -c "echo '10.42.0.10 dfallmaster' >> /etc/hosts"

# Add a new file with the default agent ID
sudo sh -c "echo $1 >> /etc/dfall_default_agent_id"

# Add a new file with the default coordinator ID
sudo sh -c "echo 1 >> /etc/dfall_default_coord_id"

# Copy rules necessary for using the Crazyradio
sudo cp ~/dfall/dfall-system/install/99-crazyflie.rules /etc/udev/rules.d
sudo cp ~/dfall/dfall-system/install/99-crazyradio.rules /etc/udev/rules.d



# --------------------------------------------------------------- #
# BUILD THE dfall-system ROS PACKAGE
# > NOTE: This is done by calling "catkin_make" from the folder "dfall_ws",
#         where "ws" stands for workspace
# > NOTE: the package that is built is named "dfall_pkg", as specified in the file
#         .../dfall_ws/src/dfall_pkg/package.xml
# > NOTE: the -j4 option specifies the: Maximum number of build jobs to be
#         distributed across active packages. (default is cpu count)
cd ~/dfall/dfall-system/dfall_ws
catkin_make -j4



# --------------------------------------------------------------- #
# ADD TO THE .bashrc THE dfall-system CONFIG AND ROS PACKAGE SETUP

# Add a heading for the addition to the .bashrc
echo "# ADDED AS PART OF THE dfall-system INSTALLATION" >> ~/.bashrc

# Add an evironment variable that point to the dfall-system repository
echo "# Environment variable for the home directory" >> ~/.bashrc
echo "export DFALL_HOME=\"/home/$USER/dfall/dfall-system\"" >> ~/.bashrc
# And set the environment variable for this terminal session
export DFALL_HOME="/home/$USER/dfall/dfall-system"

# Add the ROS environment setup to the .bashrc
echo "# Source the ROS setup" >> ~/.bashrc
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc

# Add the dfall ROS Package setup to the .bashrc
echo "# Source the dfall package setup" >> ~/.bashrc
echo "# > Note that the \"devel\" folder is created as part of running \"catkin_make\"" >> ~/.bashrc
echo "source ~/dfall/dfall-system/dfall_ws/devel/setup.bash" >> ~/.bashrc
# And run it now so that it is valid for this terminal session
source ~/dfall/dfall-system/dfall_ws/devel/setup.bash

# Add the "Config.sh" shell script to the .bashrc
echo "# Source the dfall config shell script" >> ~/.bashrc
echo "source ~/dfall/dfall-system/dfall_ws/src/dfall_pkg/launch/Config.sh" >> ~/.bashrc
# And run it now so that it is valid for this terminal session
source ~/dfall/dfall-system/dfall_ws/src/dfall_pkg/launch/Config.sh

# Add an alias for changing directory to the dfall-system repository
echo "# Alias for changing directory to the \$DFALL_HOME" >> ~/.bashrc
echo "alias dfall=\"cd $DFALL_HOME\"" >> ~/.bashrc
echo "alias cddfall=\"cd $DFALL_HOME\"" >> ~/.bashrc
# And run it now so that it is valid for this terminal session
alias dfall="cd $DFALL_HOME"
alias cddfall="cd $DFALL_HOME"