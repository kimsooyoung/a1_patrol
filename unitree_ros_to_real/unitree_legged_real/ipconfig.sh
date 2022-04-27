#!/bin/bash

# ex) lan port id was enx00e04c360310

sudo ifconfig enx00e04c360310 down
sudo ifconfig enx00e04c360310 192.168.123.162/24
sudo ifconfig enx00e04c360310 up