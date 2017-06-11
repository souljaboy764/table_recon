#!/usr/bin/python
import roslib; roslib.load_manifest('rl_nav')
import rospy

from rtabmap_ros.msg import MapData

def receiveMapData(mapData):
	print "Graph Nodes: ",
	for node in mapData.nodes:
		print node.id,
	print ""

rospy.init_node('PlannerNode')
mapData_sub = rospy.Subscriber("/rtabmap/mapData", MapData, receiveMapData)
rospy.spin()
