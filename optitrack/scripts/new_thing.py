#!/usr/bin/env python
import rospy, os

import socket

UDP_IP = "192.168.0.77"
UDP_PORT = 1512
header_b = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\" ?>"
start_string = "<CaptureStart><TimeCode VALUE=\"12 13 14 15 0 0 1 1\"/><Name VALUE=\"RemoteTriggerTest_take01\"/><Notes VALUE=""/><Description VALUE=""/><DatabasePath VALUE=\"C:\\Users\\tracking\\Downloads\"/><PacketID VALUE=\"0\"/><ProcessID VALUE=\"" +str(os.getpid()) +"\"/></CaptureStart>"

MESSAGE = header_b + start_string

print "UDP target IP:", UDP_IP
print "UDP target port:", UDP_PORT
print "message:", MESSAGE


if __name__ == '__main__':
	sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
	sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))