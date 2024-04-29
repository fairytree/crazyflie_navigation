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

# Add the teacher's IP address to the /etc/hosts file
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
catkin_make



# --------------------------------------------------------------- #
# ADD TO THE .bashrc THE dfall-system CONFIG AND ROS PACKAGE SETUP

# Add a heading for the addition to the .bashrc
echo "# ADDED AS PART OF THE dfall-system INSTALLATION" >> ~/.bashrc

# Add an evironment variable that point to the dfall-system Repository
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
