#!/usr/bin/env python
import rospy, sys, os
import optirx as rx
from struct import pack
from std_msgs.msg import String
from optitrack.utils import get_ip_address, read_parameter

version = (2, 10, 0, 0)  # the latest SDK version
motive_ip = "192.168.0.77"
iface = ""
optitrack = ""

def send(nat_value, message, some_args=""):
	if len(some_args) > 1:
		message+=", " + some_args
	message_len =  len(message) + 1
	data = pack("HH"+bytes(message_len)+"s", nat_value, 4+message_len, message)
	global optitrack, motive_ip
	optitrack.sendto(data, (motive_ip, 1510))
	print('Successfully send to optitrack')


def commandCallback(msg):
	if msg.data == "stop":
		send(2, "StartRecording")
		print('Stopped')
	elif msg.data == "":
		print("no take name given")
	else:
		send(2, "SetRecordTakeName", msg.data)
		send(2, "StartRecording")
		print('Started recording ...')

if __name__ == '__main__':
	global iface, optitrack, version
	rospy.init_node("recorder", anonymous=True)
	iface = read_parameter('~iface', 'wlp3s0')
	optitrack = rx.mkcmdsock(ip_address=get_ip_address(iface), port=1512)
	rospy.Subscriber('filename', String, commandCallback,queue_size=3)
	rospy.spin()

	