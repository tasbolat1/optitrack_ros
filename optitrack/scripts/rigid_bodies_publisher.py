#!/usr/bin/env python
import rospy, os
import numpy as np
# Optitrack
import socket
import optirx as rx
import select
from optitrack.utils import get_ip_address, read_parameter
# Transformations
import tf
import tf.transformations as tr
from hrl_geom.pose_converter import PoseConv
# Messages
from geometry_msgs.msg import Pose, Point, Quaternion
from optitrack.msg import RigidBody, RigidBodyArray
from optitrack.msg import SingleMarker, SingleMarkerArray


class RigidBodiesPublisher(object):
  """
  Naming convention for the transformations:
    - wTo: Transformation between C{parent_frame} and C{optitrack_frame}
    - oTr: Transformation between C{optitrack_frame} and the rigid body
    - wTr: Transformation between C{parent_frame} and the rigid body
  """
  def __init__(self):
    # Read parameters to configure the node
    tf_publish_rate = read_parameter('~tf_publish_rate', 50.)
    tf_period = 1./tf_publish_rate if tf_publish_rate > 0 else float('inf')
    parent_frame = read_parameter('~parent_frame', 'world')
    optitrack_frame = read_parameter('~optitrack_frame', 'optitrack')
    rigid_bodies = read_parameter('~rigid_bodies', dict())
    names = []
    ids = []
    for name,value in rigid_bodies.items():
      names.append(name)
      ids.append(value["id"])
    # Setup Publishers
    pose_pub = rospy.Publisher('/optitrack/rigid_bodies', RigidBodyArray, queue_size=3)
    pose_pub_single = rospy.Publisher('/optitrack/single_markers', SingleMarkerArray, queue_size=3)
    # Setup TF listener and broadcaster
    tf_listener = tf.TransformListener()
    tf_broadcaster = tf.TransformBroadcaster()
    # Connect to the optitrack system
    iface = read_parameter('~iface', 'eth1')
    version = (2, 10, 0, 0)  # the latest SDK version
    optitrack = rx.mkdatasock(ip_address=get_ip_address(iface))
    rospy.loginfo('Successfully connected to optitrack')
    # Get transformation from the world to optitrack frame
    (parent, child) = (parent_frame, optitrack_frame)
    try:
      now = rospy.Time.now() + rospy.Duration(1.0)
      tf_listener.waitForTransform(parent, child, now, rospy.Duration(5.0))
      (pos,rot) = tf_listener.lookupTransform(parent, child, now)
      wTo = PoseConv.to_homo_mat(pos, rot)
    except (tf.Exception, tf.LookupException, tf.ConnectivityException):
      rospy.logwarn('Failed to get transformation from %r to %r frame' % (parent, child))
      parent_frame = optitrack_frame
      wTo = np.eye(4)
    # Track up to 24 rigid bodies
    prevtime = np.ones(24)*rospy.get_time()
    # Track up to 10 single bodies
    prevtime2 = np.ones(10)*rospy.get_time()
    while not rospy.is_shutdown():
      try:	
        optitrack.setblocking(0)
        ready = select.select([optitrack], [], [], 1)
        if ready[0]:
          data = optitrack.recv(rx.MAX_PACKETSIZE)
        else:
          print("no data")
          data = ""
      except socket.error:
        if rospy.is_shutdown():  # exit gracefully
          return
        else:
          rospy.logwarn('Failed to receive packet from optitrack')
      packet = rx.unpack(data, version=version)
      
      if type(packet) is rx.SenderData:
        version = packet.natnet_version
        rospy.loginfo('NatNet version received: ' + str(version))
      if type(packet) in [rx.SenderData, rx.ModelDefs, rx.FrameOfData]:
        # Optitrack gives the position of the centroid.
        

        # my code for markers
        array_msg2 = SingleMarkerArray()
        for j, single_marker_position in enumerate(packet.other_markers):
          #print(j, single_marker_position)
          pos_opt = np.array(single_marker_position)
          rot_opt = np.array([1,0,0,0]) # unique quaternion, meaningless
          oTr = PoseConv.to_homo_mat(pos_opt, rot_opt)
          # Transformation between world frame and the rigid body
          wTr = np.dot(wTo, oTr)
          array_msg2.header.stamp = rospy.Time.now()
          array_msg2.header.frame_id = parent_frame
          body_msg2 = SingleMarker()
          pose = Pose()
          pose.position = Point(*wTr[:3,3])
          pose.orientation = Quaternion(*tr.quaternion_from_matrix(wTr))
          body_msg2.id = j
          body_msg2.pose = pose
          array_msg2.bodies.append(body_msg2)
          tf_broadcaster.sendTransform(pos_opt, rot_opt, rospy.Time.now(), "single_marker_" + str(j), optitrack_frame)
        pose_pub_single.publish(array_msg2)
        # end of my code

        # these linse for rigid body
        array_msg = RigidBodyArray()
        for i, rigid_body in enumerate(packet.rigid_bodies):
          body_id = rigid_body.id
          #print("body: ", type(rigid_body.orientation))
          pos_opt = np.array(rigid_body.position)
          rot_opt = np.array(rigid_body.orientation)
          oTr = PoseConv.to_homo_mat(pos_opt, rot_opt)
          # Transformation between world frame and the rigid body
          wTr = np.dot(wTo, oTr)
          array_msg.header.stamp = rospy.Time.now()
          array_msg.header.frame_id = parent_frame
          body_msg = RigidBody()
          pose = Pose()
          pose.position = Point(*wTr[:3,3])
          pose.orientation = Quaternion(*tr.quaternion_from_matrix(wTr))
          body_msg.id = body_id
          body_msg.tracking_valid = rigid_body.tracking_valid
          body_msg.mrk_mean_error = rigid_body.mrk_mean_error
          body_msg.pose = pose
          for marker in rigid_body.markers:
            # TODO: Should transform the markers as well
            body_msg.markers.append(Point(*marker))
          array_msg.bodies.append(body_msg)
          # Control the publish rate for the TF broadcaster
          if rigid_body.tracking_valid and (rospy.get_time()-prevtime[body_id] >= tf_period):
            body_name = 'rigid_body_%d' % (body_id)
            if body_id in ids:
              idx = ids.index(body_id)
              body_name = names[idx]
            tf_broadcaster.sendTransform(pos_opt, rot_opt, rospy.Time.now(), body_name, optitrack_frame)
            prevtime[body_id] = rospy.get_time()
        pose_pub.publish(array_msg)


if __name__ == '__main__':
  node_name = os.path.splitext(os.path.basename(__file__))[0]
  rospy.init_node(node_name)
  rospy.loginfo('Starting [%s] node' % node_name)
  opti_node = RigidBodiesPublisher()
  rospy.loginfo('Shuting down [%s] node' % node_name)
