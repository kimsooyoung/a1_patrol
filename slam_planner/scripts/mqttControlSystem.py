#!/bin/bash
import time
import os
import threading
import multiprocessing as mp
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
	client.subscribe("slam/manager")
	client.subscribe("slam/run")

def on_message(client, userdata, msg):
	print(msg.topic)
	if msg.topic == "slam/manager":
		if msg.payload.decode() == 'restart':
			print("Restart SLAM Process.") 
			os.system('./restart.sh')
		elif msg.payload.decode() == 'clear':
			print("Clear Costmap.")
			os.system('rosservice call /slam_planner_node/clear_costmaps')
		elif msg.payload.decode() == 'stop':
			print("Stop SLAM Process.")
			os.system('./stop.sh')
	elif msg.topic == "slam/run" and msg.payload =='':
		client.publish("slam/targets", '', 1, True)
		client.publish("slam/target_index", '', 1, True)
		client.publish("slam/looping", '', 1, True)

def subscriber():
	client.on_message = on_message
	client.loop_forever()

if __name__ == '__main__':	
	client = mqtt.Client()
	client.connect("192.168.123.161", 1883, 60)
	client.on_connect = on_connect
	
	p1 = mp.Process(target=subscriber)
	p1.start()
