#!/bin/bash

# sudo ifconfig lo multicast
# sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

# sudo ifconfig enp2s0 down
# sudo ifconfig enp2s0 up 192.168.123.162 netmask 255.255.255.0

sudo ifconfig eth0 down
sudo ifconfig eth0 192.168.123.162/24
sudo ifconfig eth0 up