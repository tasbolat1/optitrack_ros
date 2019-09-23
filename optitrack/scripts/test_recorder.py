#!/usr/bin/env python
import rospy, os

import socket
import optirx as rx
from struct import pack

from optitrack.utils import get_ip_address, read_parameter

if __name__ == '__main__':

	# Connect to the optitrack system
	rospy.init_node("recorder")
	iface = read_parameter('~iface', 'wlp3s0')
	version = (2, 10, 0, 0)  # the latest SDK version
	optitrack = rx.mkcmdsock(ip_address=get_ip_address(iface), port=1512)
	#optitrack = rx.mkcmdsock(ip_address="192.168.0.77", port=5050)


# 	start_string="<CaptureStart>\
#     <Name VALUE=\"RemoteTriggerTest_take01\"/>\
#     <SessionName VALUE=\"SessionName\" />\
#     <Notes VALUE=\"\"/>\
#     <Assets VALUE=\"RigidBody1\" />\
#     <Description VALUE=\"\" />\
#     <DatabasePath VALUE=\"\"/>\
#     <TimeCode VALUE=\"00:00:00:00\"/>\
#     <PacketID VALUE=\"0\"/>\
# </CaptureStart>"
	#header_b = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>"
	#start_string = "<CaptureStart><TimeCode VALUE=\"12 13 14 15 0 0 1 1\"/><Name VALUE=\"RemoteTriggerTest_take01\"/><Notes VALUE=""/><Description VALUE=""/><DatabasePath VALUE=\"C:Users\\tracking\\Downloads\"/><PacketID VALUE=\"0\"/><ProcessID VALUE=\"" +str(os.getpid()) +"\"/></CaptureStart>"
	#NAT_REQUEST = 2
	#commandStr = "StartRecording"
	# packetSize = len(commandStr) + 1
	# data = pack("HH", packetSize)
	# data += packetSize.to_bytes( 2, byteorder='little' )
	# data += commandStr.encode( 'utf-8' )
	# data += b'\0'


	#message = "StartRecording"
	#message = "StopRecording"
	message = "SetRecordTakeName, new_take4"
	nat_value = 2
	message_len = len(message) + 1
	data = pack("HH"+bytes(message_len)+"s", nat_value, 4+message_len, message)

	print(optitrack.sendto(data, ("192.168.0.77", 1510)))

	message = "StartRecording"
	#message = "StopRecording"
	#message = "SetRecordTakeName, new_take3"
	nat_value = 2
	message_len = len(message) + 1
	data = pack("HH"+bytes(message_len)+"s", nat_value, 4+message_len, message)

	print(optitrack.sendto(data, ("192.168.0.77", 1510)))
	#optitrack.send(start_string)

	print('Successfully connected to optitrack')