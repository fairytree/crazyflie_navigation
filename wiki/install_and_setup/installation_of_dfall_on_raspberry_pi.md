# Raspberry Pi Installation


On this page:
- [Install ROS](#install-ros)
- [Install the python USB package](#install-the-python-USB-package)
- [Install the apache web server](#install-the-apache-web-server)
- [Install php](#install-php)
- [Clone the ``dfall-system`` repository](#clone-the-dfall-system-repository)
- [Allow git and catkin make over the web interface](#allow-git-and-catkin-make-over-the-web-interface)
- [Add the ``www-data`` user to the ``plugdev`` group](#add-the-www-data-user-to-the-plugdev-group)
- [Copy across the ``99 crazy rules``](#copy-across-the-99-crazy-rules)
- [Make the ``www-data`` user the owner of the ``/var/www/`` folder](#make-the-www-data-user-the-owner-of-the-var-www-folder)
- [Copy across the website](#copy-across-the-website)
- [Useful commands](#useful-commands)



## Install ROS

Download the appropriate installation script using the following command:
```
dfall_install_ros_melodic_keys.sh
```

Change the permissions of the file to make it executable using the command:
```
chmod +x dfall_install_ros_melodic_keys.sh
```

Run the installation script using the command:
```
./dfall_install_ros_melodic_keys.sh
```

This file only has two commands that are tedious to write out, but if you are patient then you can skip the above three commands and enter the following two commands:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

The first command sets up the computer to accept software from ``packages.ros.org``, and the second command sets up the keys.


Now install ROS Melodic using the following command:
```
sudo apt -y install ros-melodic-ros-base
```
Note, we install "ROS-Base: (Bare Bones)" because ubuntu server is a "headless" operating system, and hence does not require the graphical desktop feature of the "Desktop-Full" ROS installation.

Initialise and update rosdep using the following commands:
```
sudo rosdep init
rosdep update
```
``rosdep`` enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS.

If desired add the ROS environment setup to the ``.bashrc`` using the following command:
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
```



## Install the python USB package

This is needed to connected to the Crazyradio USB dongle.

Install the python package management system ``python-pip`` using the following command:
```
sudo apt -y install python-pip
```

Install the python USB package ``pyusb`` using the following command:
```
sudo pip install pyusb
```



## Install the apache web server

The apache web server can be installed with the following command:
```
sudo apt install apache2
```



## Install php

Php can be installed with the following command:
```
sudo apt install php
```



## Clone the ``dfall-system`` repository

The web interface allows the user to upload certain files to the ``dfall-system`` repository and also to perform ``git`` and ``catkin_make`` actions on the repository. Hence the ``www-data`` requires write access to folder where the ``dfall-system`` repository is cloned. We choose to clone the repository into the base location ``/home/www-share`` as this is a natural location for data that can have accessed share between ``www-data`` and other users of the ubuntu server installation, namely the ``ubuntu`` user.

Create a shared folder using the follow sequence of commands:
```
cd /home
sudo mkdir www-share
sudo chmod 777 www-share
cd www-share
sudo -u www-data mkdir dfall
sudo -u www-data chmod 775 dfall
```

Clone the repository as ``the www-data`` user using the following command
```
cd /home/www-share/dfall
sudo -u www-data git clone https://gitlab.ethz.ch/dfall/dfall-system
```



## Allow git and catkin make over the web interface

Open the ``/etc/sudoers`` file for editing:
```
sudo nano /etc/sudoers
```
Add the following line to the ``/etc/sudoers`` file that allows the ``www-data`` user to execute ``git pull`` commands:
```
www-data ALL=(www-data) /usr/bin/git pull
```
Note, this is line is most naturally included under the ``User privilege specification`` comment in the ``/etc/sudoers``, i.e.:
```
# User privilege specification
root    ALL=(ALL) ALL
git ALL=(www-data) ALL
```



## Add the ``www-data`` user to the ``plugdev`` group

Add the ``www-data`` user to the ``plugdev`` group using the following command:
```
sudo usermod -a -G plugdev www-data
```
NOTE: this is the group nominated in the ``udev rules`` for the CrazyRadio (see the ``install`` folder). This allows a CrazyRadio node that is launched by the web interface to access the CrazyRadio USB dongle.

To confirm the group allocation, view the ``group`` file using the command:
```
less /etc/group
```

If you need to remove a user from a group, then use the command:
```
deluser www-data plugdev
```
where the syntax here is: ``deluser <username> <groupname>``


## Copy across the ``99 crazy rules``

The following commands copy the rules necessary for using the Crazyradio:
```
sudo cp /home/www-share/dfall/dfall-system/install/99-crazyflie.rules /etc/udev/rules.d
sudo cp /home/www-share/dfall/dfall-system/install/99-crazyradio.rules /etc/udev/rules.d
```



## Make the ``www-data`` user the owner of the ``/var/www/`` folder

This is required so that the web interface can be updated by calling a php script that deletes all the contents of the ``/var/www/html/`` folder and copies the web interface files from the ``dfall-system`` repository that was cloned above to the location ``/home/www-share/dfall``.

Enter the following commands:
```
sudo chown -R www-data /var/www
sudo chgrp -R www-data /var/www
sudo chmod -R g+w /var/www
```



## Copy across the website

Use the following command to copy all the web interface files from the ``dfall-system`` repository (cloned above) to the ``/var/www/html`` folder from which the web interface is hosted:
```
sudo -u www-data cp -R /home/www-share/dfall/dfall-system/web_interface/html/* /var/www/html/
```



## Useful commands

The apache web server can be ``{stop,start,restart,reload}`` using the ``systemctl`` command as follows:
```
sudo systemctl stop apache2.service
sudo systemctl start apache2.service
sudo systemctl restart apache2.service
sudo systemctl reload apache2.service
```
