# Network Setup
## Setting up the Vicon network
Inseret the Ethernet cable from Vicon into your computer and go to network settings in the top right corner. Click on _Edit Connections..._ <br>
Choose the Ethernet connection (probably named _wired connection 1_) and click on _edit_.
<br><img src="./pics/setup_pics/viconNetwork1.png" style="width: 500px;"/> <br>
Change the connection name to _Vicon_.
<br><img src="./pics/setup_pics/viconNetwork2.png" style="width: 500px;"/> <br>
Then go to the _IPv4 Settings_ and choose **Manual** as the _Method_ and then add your IP address provided by your teacher (Form 10.42.0.xx). Click tab twice and _save_.
<br><img src="./pics/setup_pics/viconNetwork3.png" style="width: 500px;"/> <br>
**Don't forget to choose the Vicon network manually if you choose to enable WiFi and Ethernet simultaneously as described below!**

<br>

## Vicon, teacher and students
During installation process is the IP address of the teacher set to 10.42.0.10. (This value is written to the /etc/hosts file such that this IP address is accessible through the keyword _dfallmaster_) <br>
Have a look at `Config.sh` in `~/dfall_ws/src/dfall_pkg/launch/`
<br><img src="./pics/setup_pics/configsh2.png" style="width: 500px;"/> <br>
Here you see, that the ROS Master URI is set to be the teacher. This means that _roscore_ runs only on the teacher's computer. Your own IP address (_ROS IP_) is also set and taken from your Ethernet settings as defined in the section _Setting up the Vicon network_.
<br>

## IP-Addresses
Currently the teacher's IP is ``10.42.0.10`` and the student's IP are of the format ``10.42.0.xx``, where xx is an unused address.

## Using WLAN and Vicon simultaneously
Without some adjustments it is not possible to use an Ethernet and a wireless network at the same time. Vicon is connected via cable and therefore wouldn't allow a connection to the Internet. Because it's tedious to always unplug the cable just to be able to look something up on Google, we provide an explanation on how to enable simultaneous usage of WLAN and Vicon. <br>
**But careful: If you enable this setting you have to choose the Vicon network manually. This is a common mistake! You will get an error if the cable is inserted but not manually chosen as shown in the picture below.**
<br><img src="./pics/setup_pics/chooseVicon.jpg" style="width: 500px;"/> <br>
### Step by Step:
Ubuntu allows multiple connections by default, but sometimes, we need to specify which one to use. Here we use LAN for the Intranet and WiFi for the Internet.

So, firstly search for Network Connections in the unity dash. Then, under the Ethernet section, click 'Add' button.
<br><img src="./pics/setup_pics/internetethernet1.png" style="width: 500px;"/> <br>

Then, we need to create a new Ethernet connection which we are going to enable manually. Uncheck the option of connecting automatically since this is your Intranet connection. **Choose _Vicon_ as a name instead of Intranet**
<br><img src="./pics/setup_pics/internetethernet2.png" style="width: 500px;"/> <br>

Then go to the IPv4/IPv6 settings (depending on your network) and then click on the Routes button. Check the 'Use this connection only for resources on its network' option. Click Save.
<br><img src="./pics/setup_pics/internetethernet3.png" style="width: 500px;"/> <br>
Now you can use both the LAN and WiFi simultaneously.

<br>
Source: <br> https://askubuntu.com/questions/639100/how-to-get-connection-to-both-wifi-as-well-as-lan-in-ubuntu-14-04-lts#639425
<br>
Some additional informations: <br> http://aleksz-programming.blogspot.ch/2013/01/using-wifi-and-network-cable-at-same.html
