# Install the apache web server
sudo apt install apache2

# Install php
sudo apt install php

# DISABLE iPv6
# Taken from:
# https://www.configserverfirewall.com/ubuntu-linux/ubuntu-disable-ipv6/

# Permanently Disable IPv6 on Ubuntu 18.04/16.04 with sysctl

# Easiest and safest method is to add configurations to the /etc/sysctl.conf file. To disable IPv6 using sysctl, Open the Ubuntu terminal and Perform the following steps:

# Open the /etc/sysctl.conf file:

# vim /etc/sysctl.conf

# Add the following lines at the end of the sysctl.conf file:

# net.ipv6.conf.all.disable_ipv6 = 1
# net.ipv6.conf.default.disable_ipv6 = 1
# net.ipv6.conf.lo.disable_ipv6 = 1

# In Ubuntu server 18.04, you will need to add additional lines for each interface you want to disable IPv6:

# net.ipv6.conf.<ifname>.disable_ipv6 = 1

# For example, if the interface name is enp0s3, Then:

# net.ipv6.conf.enp0s3.disable_ipv6 = 1

# For change to be effected, run the sysctl -p command.

# sysctl -p

# Then, run the following command to check the IPv6 status:

# cat /proc/sys/net/ipv6/conf/all/disable_ipv6

# If the output is 1 then IPv6 is disabled, the command will output 0 when IPv6 is enabled.

# If you want to re enable IPv6 addresses, remove the above configuration from the sysctl.conf and execute the sysctl -p command.




# CRTICAL TO GET THE gateway4 and nameserver correct
# for example 192.168.0.1




# Create a shared folder
cd /home
sudo mkdir www-share
sudo chmod 777 www-share
cd www-share
sudo -u www-data mkdir dfall
sudo -u www-data chmod 775 dfall

# Clone the repository
# NOTE: very important here is that the repository
#       is cloned as the www-data user
cd /home/www-share/dfall
sudo -u www-data git clone https://gitlab.ethz.ch/dfall/dfall-system

# Add the necessary line to the "/etc/sudoers" file
# that allows the "www-data" user to execute
# "git pull" commands
# >> www-data ALL=(www-data) /usr/bin/git pull

# Add the "www-data" user to the "plugdev" group
# NOTE: this is the group nominated in the udev
#       rules for the CrazyRadio (see the "install"
#       folder). This allows a CrazyRadio node
#       that is launched by the web interface to
#       access the USB dongle.
sudo usermod -a -G plugdev www-data

# To confirm the group allocation, view the file:
# >> less /etc/group

# If you need to remove a user from a group, then:
# SYNTAX: >> deluser <username> <groupname>
# >> deluser www-data plugdev

# DON'T FORGOT TO COPY ACROSS THE 99.crazy.rules


# www-data must own the /var/www/ folder
sudo chown -R www-data /var/www
sudo chgrp -R www-data /var/www
sudo chmod -R g+w /var/www

# Copy across the website
sudo -u www-data cp -R <dfall-rep>/web_interface/html/* /var/www/html/



# ============================================ #
# USEFUL COMMANDS
#
# The apache web server can be
# {stop,start,restart,reload}
# using the "systemctl" command as follows:
#
#sudo systemctl stop apache2.service
#sudo systemctl start apache2.service
#sudo systemctl restart apache2.service
#sudo systemctl reload apache2.service