# Raspberry Pi Installation

On this page:
- [Enable responding to broadcast pings](#enable-responding-to-broadcast-pings)
- [Permanently disable IPv6](#permanently-disable-ipv6)
- [Setup the ethernet network connection](#setup-the-ethernet-network-connection), with the following configurations detailed on this page:
  - [Default configuration of a fresh Ubuntu install](#default-configuration-of-a-fresh-ubuntu-install)
  - [Configuration when using a desktop environment](#configuration-when-using-a-desktop-environment)
  - [Configuration for a fixed IP address](#configuration-for-a-fixed-ip-address)
  - [Configuration for a dynamic IP address](#configuration-for-a-dynamic-ip-address)
  - [Useful commands for debugging network discrepancies](#useful-commands-for-debugging-network-discrepancies)



## ENABLE RESPONDING TO BROADCAST PINGS

Taken from: https://www.theurbanpenguin.com/broadcast-icmp-in-linux-and-how-to-initiate-and-protect/

This makes it significantly easier to discover the IP address of the RaspberryPi when it is connected to a network, especially so if the [dynamic IP address](#configuration-for-a-dynamic-ip-address) network configuration is chosen.

Open the ``/etc/sysctl.conf`` file for editing:
```
sudo nano /etc/sysctl.conf
```
Add the following lines at the end of the ``sysctl.conf`` file:
```
net.ipv4.icmp_echo_ignore_all=0
net.ipv4.icmp_echo_ignore_broadcasts=0
```

To make the change take effect, enter the command:
```
sysctl -p
```
Then, enter the following command to check the "ignore broadcast" status:
```
less /proc/sys/net/ipv4/icmp/echo_ignore_broadcasts
```
If the output is 1 then broadcast pings are ignore, otherwise, if the output 0 then broadcast pings will be reponded to.

To again ignore broadcast pings, simply remove the above changes made to the ``sysctl.conf`` file and then enter the ``sysctl -p`` command.



## PERMANENTLY DISABLE IPv6

Taken from: https://www.configserverfirewall.com/ubuntu-linux/ubuntu-disable-ipv6/

It is not necessary to disable IPv6, but some forums mention that having IPv6 can cause unexpected network behaviour under certain circumstancs.

Open the ``/etc/sysctl.conf`` file for editing:
```
sudo nano /etc/sysctl.conf
```
Add the following lines at the end of the ``sysctl.conf`` file:
```
net.ipv6.conf.all.disable_ipv6 = 1
net.ipv6.conf.default.disable_ipv6 = 1
net.ipv6.conf.lo.disable_ipv6 = 1
```

To make the change take effect, enter the command:
```
sysctl -p
```
Then, enter the following command to check the IPv6 status:
```
less /proc/sys/net/ipv6/conf/all/disable_ipv6
```
If the output is 1 then IPv6 is disabled, otherwise, if the output 0 then IPv6 is enabled.

To re enable IPv6 addresses, simply remove the above changes made to the ``sysctl.conf`` file and then enter the ``sysctl -p`` command.

## SETUP THE ETHERNET NETWORK CONNECTION

The networking is handled by ``netplan`` and hence network specifications are contained in the ``/etc/netplan`` folder. A comprehensive set of example configurations are available here:
<br>
https://netplan.io/examples



### Default configuration of a fresh Ubuntu install

As installed, the ``/etc/netplan`` folder should contain only one file named ``50-clound-init.yaml``, and the contents of the file should look something like:

```
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: true
            match:
                macaddress: xx:xx:xx:xx:xx:xx
            set-name: eth0
```

You should change this file depending on the network to which the RaspberryPi will be connected. Examples are provided below for common setup options.

After you have edited the file, using for example ``sudo nano /etc/netplan/50-clound-init.yaml``, then you can make the changes active by first validating the changes using the command:
```
sudo netplan try
```

This should return a message informing you whether the new configuration is valid or not. If the configuration is not valid, then the system reverts to the previous settings, and the likely cause of the error is a typing mistake.

Once validated, apply the changes using the following command:
```
sudo netplan apply
```



### Configuration when using a desktop environment

If using a desktop environment, then you specify that the ``network-manager`` provided by the desktop is in charge of managing connections by editing the ``50-clound-init.yaml`` file to be the following:
```
network:
    version: 2
    renderer: NetworkManager
```

Don't forget to ``try`` and ``apply`` the changes as described above.


### Configuration for a fixed IP address

If you did not install a desktop environment, i.e., you are running "headless" Ubuntu, then you can set a fixed IP address by editing the ``50-clound-init.yaml`` file to be the following:
```
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: false
            addresses: [10.42.0.11/24]
            gateway4: 10.42.0.0
            nameservers:
                addresses: [8.8.8.8,8.8.4.4]
```

The address ``10.42.0.11/24`` should be replaced by the IP address and netmask you desire. The ``/24`` specifies the most comment netmask of ``255.255.255.0``. It is also possible to leave out the ``gateway4`` and ``nameservers`` specifications if the defaults are appropriate.

If the RaspberryPi is plugged into a home router then you should choose the fixed IP address to match the domain of the router. For example, if IP addresses on your home network take the form ``192.168.1.xxx`` where ``xxx`` is a number that addresses each device on the network. In this case it is important to specify the ``gateway4`` as the address of the router, typically with a ``1`` at the end and hence ``192.168.1.1`` in this example. It is also important to add the router's address to the ``nameservers: addresses``. For this example, the ``50-clound-init.yaml`` would look something like the following:
set a fixed IP address by editing the ``50-clound-init.yaml`` file to be the following:
```
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: false
            addresses: [192.168.1.11/24]
            gateway4: 192.168.1.1
            nameservers:
                addresses: [192.168.1.1,8.8.8.8,8.8.4.4]
```

Don't forget to ``try`` and ``apply`` the changes as described above.



### Configuration for a dynamic IP address

To set the RaspberryPi to obtain an IP address dynamically from the host to which it is connected, then edit the ``50-clound-init.yaml`` file to be the following:
```
network:
    version: 2
    ethernets:
        eth0:
            dhcp4: true
```

The challenge when using this configuration is to find the IP address of the RaspberryPi once it is connected to the network.


Don't forget to ``try`` and ``apply`` the changes as described above.


### Useful commands for debugging network discrepancies

The following are some useful commands for checking the current connection status:
```
ifconfig
```

```
nmcli c show
```
