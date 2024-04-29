# Raspberry Pi Installation

On this page:
- [Installing Ubuntu server](#installing-ubuntu-server)
- [Standard configuration steps for Ubuntu server](#standard-configuration-steps-for-ubuntu-server)



## INSTALLING UBUNTU SERVER

### Create an SD card with the pre-installed Ubuntu serve image

This step is to flash the Ubuntu 18.04 image onto the SD card for the Raspberry Pi. The following instructions are taken from the Raspberry Pi website:
https://www.raspberrypi.org/documentation/installation/installing-images/


Download the latest Ubuntu 18.04 (server version) from either of these two Ubuntu pages:
https://www.ubuntu.com/download/iot/raspberry-pi-2-3
https://wiki.ubuntu.com/ARM/RaspberryPi

Uncompress the downloaded file using the appropriate tool available on your operating system (Windows: 7-Zip, Mac: The Unarchiver, Linux: Unzip)


Proceed as follows for Mac:

Insert the SD card into your computer, open the Disk Utility application, and format the SD card with options:
Format: MS-DOS (FAT)
Scheme: Master Boot Record

Open terminal and use the command ``diskutil list`` to identify which disk corresponds to the SD card, where you are looking for the label of the form ``disk2`` and NOT the label of the form ``disk2s1``

Unmount all partitions of the SD card using the command:
```
diskutil unmountDisk /dev/disk<disk# from diskutil>
```

Copy the Ubuntu image to the SD card using the command:
```
sudo dd bs=1m if=~/Downloads/image.img of=/dev/rdisk<disk# from diskutil> conv=sync
```

The copying will take a few minutes and not display any status information. It is possible to retrieve the current progress by pressing ``Ctrl+T`` to send the ``SIGINFO`` signal to the respective terminal where the command is running.

Once the copy has completed, eject the SD card using the command:
```
sudo diskutil eject /dev/rdisk<disk# from diskutil>
```



### Change the boot-loader for RaspberryPi 3 B+
Insert the SD card into your computer and open the ``config.txt`` file in your favourite editor. Adjust the lines:
```
kernel=...
device_tree_address=0x02000000
```
to instead be
```
kernel=vmlinuz
initramfs initrd.img followkernel
#device_tree_address=0x02000000
```

This step was taken from:
https://www.raspberrypi.org/forums/viewtopic.php?t=233794



### First boot of Ubuntu on the RaspberryPi

This step is to insert the SD card into the Raspberry Pi, boot it up, and setup the Ubuntu installation. For this first boot it is required to have a wired internet connection that does not require any authentication.

After booting you are prompted to enter the username and password, which are ``ubuntu`` and ``ubuntu`` respectively. You are then asked to change the password.

After entering the new password there is sometimes the error ``Authentication token manipulation error``, the screen clears, and you are again prompted for the username and password. This error relates to some problem with the write access for setting the new password. To overcome this error, shutdown the Raspberry Pi (simply unplug the power), remove the SD card and insert the SD card into your computer. Open the ``cmdline.txt`` using your favourite editor and add the following at the end of the line:
```
 init=/bin/sh
```

NOTE: this ``cmdline.txt`` file should be use one line of text, and each command should separated by a space. So if the file originally looks like:
```
previous content of file
```

Then the file after editing should look like:
```
previous content of file init=/bin/sh
```

Put the SD card back into the Raspberry Pi and power it up. A few boot steps are performed and then you are presented with a cursor. At the prompt type the following command (if you are presented with an error message, then simply type the command again):
```
mount -o remount, rw /
```

The results should be a message something like:
```
EXT4-fs (mmcblk0p2): re-mounted. 0pts: (null)
```

Now update the password for the ubuntu user using the command:
```
passwd ubuntu
```

You are prompted to enter the new password twice, and then the following message should be displayed:
```
passwd: password updated successfully
```

Now type the following commands:
```
sync
exec /sbin/init
```

The Raspberry Pi now continues to boot normally, and you can log into with the username ``ubuntu`` and the new password you set.

Shut down the Raspberry Pi using the command:
```
sudo halt
```

Remove the SD card from the Raspberry Pi, insert the SD card into your computer, open the ``cmdline.txt`` file again, and remove the ``init=/bin/sh`` that you added.

Eject the SD card from your computer, insert the SD card into the Raspberry Pi, and you are good to go with your new password and normal booting.



These steps to reset the password were taken from:
https://www.raspberrypi-spy.co.uk/2014/08/how-to-reset-a-forgotten-raspberry-pi-password/



## STANDARD CONFIGURATION STEPS FOR UBUNTU SERVER

### Upgrade all software

This may be started automatically, and when you try the commands:
```
sudo apt update
sudo apt list --upgradable
sudo apt upgrade
```

you may get the error:
```
E: Could not get lock /var/lib/dpkg/lock - open (11 Resource temporarily unavailable)
E: Unable to lock the administration directory (/var/lib/dpkg/) is another process using it?
```

The answer is yes, another process is using ``dpkg`` and hence it is locked, and it is likely that the process is the automatic daily apt upgrade service. To list all the automatic timers managed by the system, enter the command:
```
systemctl list-timers
```

On the list you are likely to see ``apt-daily.timer`` and ``apt-daily-upgrade.timer``. Alternatively, to check if the timers, and the services they trigger, are active, enter the following commands:
```
systemctl is-active apt-daily.timer
systemctl is-active apt-daily.service
systemctl is-active apt-daily-upgrade.timer
systemctl is-active apt-daily-upgrade.service
```

Another way to check if the ``apt`` process is running to use ``grep`` to search the list of all processes that are running. This is done with the following command:
```
ps aux | grep -i apt
```
This will display a list of the processes running ``apt`` or ``apt-get``. Note that you can ignore the process containing ``grep --color=auto``

If the services are active, then it is recommended to allow time for them to complete, and then disable the services and timers with the following commands:
```
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily.timer
sudo systemctl disable apt-daily-upgrade.service
sudo systemctl disable apt-daily-upgrade.timer
```

Auto updates and upgrade settings are also specified in the file ``/etc/apt/apt.conf.d/20auto-upgrades``. Edit this file using:
```
sudo nano /etc/apt/apt.conf.d/20auto-upgrades
```
so that the update and upgrade options are set to a value of ``0``, i.e., the contents of the file should be:
```
APT::Periodic::Update-Package-Lists "0";
APT::Periodic::Unattended-Upgrade "0";
```


### Correct the error: ``failed to start load kernel modules``

While booting, one item is listed in red as ``FAILED`` with the message:
```
Failed to start Load Kernel Modules
```

Once logged in, run the following command to list the various system ``units`` and their status:
```
systemctl list-units
```

In the list you should see the unit ``system-modules-load.service`` that is described as ``Load Kernel Modules`` and is indicated as ``loaded`` but the ``active`` and ``sub`` flags are both ``failed``. You can get more information about this error by using the command:
```
systemctl status systemd-modules-load.service
```

The status that is printed out will likely indicate the error as:
```
Failed to find module ib_iser
```

Looking through various forums posts about this error message, it seems the best course of action is to edit the file:
```
nano /lib/modules-load.d/open-iscsi.conf
```

and comment out the ``ib_iser`` line, i.e., change the line to ``#ib_iser``. You can then test if this resolves the error by either a ``reboot``, or by restarting the service and then checking its status with the following commands:
```
systemctl restart systemd-modules-load.service
systemctl status systemd-modules-load.service
```

Note that even if left un-resolved, this error does not prevent the installation and running of ROS-Melodic and the dfall-system software.



### Creating and enabling a swap file

Whether or not you plan to run resource hungry software (for example web-browsing), it can be useful to create a swap file partition to avoid out-of-memory errors. Although using a swap file for memory is much slower than the on-board RAM, and may cause the SD card to wear out quicker, it does avoid a system crash during those operations that require high memory.

Create the swap file with the following commands
```
sudo fallocate -l 1G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

Specify that the swap file is to be use by adding the following entry to ``/etc/fstab``:
```
/swapfile swap swap defaults 0 0
```

Using ``sudo nano /etc/fstab`` is an easy way to edit the file, and using ``less /etc/fstab`` is an easy way to check that the edits were correctly saved.


Reboot the Raspberry Pi (simply use the command ``reboot``). And upon rebooting, check that the swap file is available using the command:
```
sudo swapon --show
```

The output should look something like:
```
NAME      TYPE  SIZE USED PRIO
/swapfile file 1024M   0B   -2
```

This step was also taken from:
https://www.raspberrypi.org/forums/viewtopic.php?t=233794



### Removing the cloud init server capability

As the Ubuntu image is a server image, it comes pre-installed with cloud-init. The boot time of the Raspberry Pi can be slower due to cloud-init, so if you do not plan to actually use the Raspberry Pi as a server, then cloud-init can be safely removed with the following commands:

```
sudo rm -rf /etc/cloud/
sudo apt purge cloud-init
```

This step was also taken from:
https://www.raspberrypi.org/forums/viewtopic.php?t=233794



### Install a graphical desktop interface

For example:
```
sudo apt-get install lubuntu-desktop
```

Other desktops are available, for example ``xubuntu-desktop`` and ``kubuntu-desktop``, but the dfall-system has only been tested to work with ``lubuntu-desktop``. Note that this can take up to an hour to install.




### Add repositories for main, universe, multiverse, and restricted

These should already be added by default, but you can check they are added by simply trying to add them (again) using the following commands:
```
sudo add-apt-repository main
sudo add-apt-repository universe
sudo add-apt-repository multiverse
sudo add-apt-repository restricted
```

If any of these were not already added, then run ``sudo apt update`` and ``upgrade``.
