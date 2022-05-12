#!/bin/bash

# ex) lan port id was enx00e04c360310

# eth0:0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
#         inet 192.168.11.110  netmask 255.255.255.0  broadcast 192.168.11.255
#         ether 48:b0:2d:07:8d:9b  txqueuelen 1000  (Ethernet)
#         device interrupt 37  

# gedit /etc/network/interfaces

# auto eth0:0
# iface eth0:0 inet static
# name Ethernet alias LAN card
# address 192.168.11.110
# netmask 255.255.255.0
# broadcast 192.168.11.255
# network 192.168.11.1

sudo ifconfig enx00e04c360310 down
sudo ifconfig enx00e04c360310 192.168.123.162/24
sudo ifconfig enx00e04c360310 up

sudo ifconfig eth1 down
sudo ifconfig eth1 192.168.11.100
sudo ifconfig eth1 192.168.11.110
sudo ifconfig eth1 up