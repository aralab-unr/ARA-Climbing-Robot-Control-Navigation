#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from _classes import Vertice, FindEdge, Surfaces, Edge
from aStar import Star
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Pose, Vector3
from tf2_geometry_msgs import PointStamped, PoseStamped
import tf2_ros 
from climbing_robot.msg import Path
import numpy as np
from nav_msgs.msg import Odometry

points = []
verticeGraph = {}
id = 0
# targetmsg = Point()

# def targetCallback(msg):
# 	global targetmsg
# 	targetmsg = msg

def findEndTransform(end, targetNode, tfBuffer, frame_id):
	# if target node is not on the ground use the frame of its surface, otherwise use base frame
	if not targetNode.ground:
		end_frame_pos = tfBuffer.transform(end.pos, targetNode.surface.id)
		target_frame_pos = targetNode.frame_pos
	else:
		end_frame_pos = end.pos
		target_frame_pos = targetNode.pos

	endA = np.array((end_frame_pos.point.x, end_frame_pos.point.y, end_frame_pos.point.z))
	targetNodeA = np.array((target_frame_pos.point.x, target_frame_pos.point.y, target_frame_pos.point.z))
	# find distance vector between start and node
	ed = endA - targetNodeA
	# find angle between target node and end, relative to the set frame
	e_yaw = np.arctan2(ed[1],ed[0])
	# create z and w values for rotation quaternion
	ez = np.sin(e_yaw/2)
	ew = np.cos(e_yaw/2)
	e = PoseStamped()
	e.pose = Pose(end.pos.point, Quaternion(0,0,ez,ew))
	e.header.frame_id = targetNode.surface.id
	# transform pose into the base frame
	# e = tfBuffer.transform(e, 'robot_odom_frame', rospy.Duration(10))
	e = tfBuffer.transform(e, 'camera_odom_frame', rospy.Duration(10))
	et = TransformStamped()
	et.transform.translation = Vector3(end.pos.point.x, end.pos.point.y, end.pos.point.z)
	et.transform.rotation = e.pose.orientation
	# et.header.frame_id = 'robot_odom_frame'
	et.header.frame_id = 'camera_odom_frame'
	et.child_frame_id = str(frame_id)
	return et

def findStartTransform(start, minS, tfBuffer):
	# if starting node is not on the ground use the frame of its surface, otherwise use base frame
	if not minS.ground:
		start_frame_pos = tfBuffer.transform(start.pos, minS.surface.id)
		minS_frame_pos = minS.frame_pos
	else:
		start_frame_pos = start.pos
		minS_frame_pos = minS.pos 

	startA = np.array((start_frame_pos.point.x, start_frame_pos.point.y, start_frame_pos.point.z))
	minSA = np.array((minS_frame_pos.point.x, minS_frame_pos.point.y, minS_frame_pos.point.z))
	# find distance vector between start and node
	sd = startA - minSA
	# find angle between start and node, relative to the set frame
	s_yaw = np.arctan2(sd[1],sd[0])
	# create z and w values for rotation quaternion
	sz = np.sin(s_yaw/2)
	sw = np.cos(s_yaw/2)
	s = PoseStamped()
	s.pose = Pose(start.pos.point, Quaternion(0,0,sz,sw))
	s.header.frame_id = minS.surface.id
	# transform pose into the base frame
	# s = tfBuffer.transform(s, 'robot_odom_frame', rospy.Duration(10))
	s = tfBuffer.transform(s, 'camera_odom_frame', rospy.Duration(10))
	st = TransformStamped()
	st.transform.translation = Vector3(start.pos.point.x, start.pos.point.y, start.pos.point.z)
	st.transform.rotation = s.pose.orientation
	# st.header.frame_id = 'robot_odom_frame'
	st.header.frame_id = 'camera_odom_frame'
	st.child_frame_id = '1'
	return st

def findPath(msg,tfBuffer):
	# initialize publishers
	path_pub = rospy.Publisher("generated_path", Path, queue_size=10)
	vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
	broadcaster = tf2_ros.StaticTransformBroadcaster()

	S = Surfaces()
	global id
	
	# find current pose of robot
	# current_pose = tfBuffer.lookup_transform('robot_pose_frame','robot_odom_frame', rospy.Time())
	rospy.wait_for_message('/camera/odom/sample', Odometry, timeout=5)
	# current_pose = tfBuffer.lookup_transform('robot_pose_frame','robot_odom_frame', rospy.Time())
	current_pose = tfBuffer.lookup_transform('camera_pose_frame','camera_odom_frame', rospy.Time())

	# create path finding object
	findPath = Star()

	# set starting vertice to current position of robot
	start = Vertice()
	start.pos.point = Point(current_pose.transform.translation.x, current_pose.transform.translation.y, current_pose.transform.translation.z - 0.19)
	start.id = 'start'
	# start.pos.header.frame_id = 'robot_odom_frame'
	start.pos.header.frame_id = 'camera_odom_frame'

	# set goal vertice to Target message
	end = Vertice()
	end.pos.point = msg
	end.pos.point.z = end.pos.point.z - 0.19
	# end.pos.header.frame_id = 'robot_odom_frame'
	end.pos.header.frame_id = 'camera_odom_frame'
	end.id = 'end'


	startA = np.array((start.pos.point.x, start.pos.point.y, start.pos.point.z))
	endA = np.array((end.pos.point.x, end.pos.point.y, end.pos.point.z))
	mindS = 500
	mindE = 500
	minS = Vertice()
	minS.pos.point = Point(500,500,500)
	minE = Vertice()
	minE.pos.point = Point(500,500,500)

	# find nodes closest to start and end
	for i in points:
		a = np.array((i.pos.point.x,i.pos.point.y,i.pos.point.z))
		distS = np.linalg.norm(startA - a)
		distE = np.linalg.norm(endA - a)
		if distS < mindS:
			mindS = distS
			minS = i
		if distE < mindE:
			mindE = distE
			minE = i

	# find neighbor of node closest to goal that is closest to the starting position
	targetNode = minE
	for node in verticeGraph[minE.id]:
		targetNodeA = np.array((targetNode.pos.point.x, targetNode.pos.point.y, targetNode.pos.point.z))
		targetNodeD = np.linalg.norm(startA - targetNodeA)
		nodeA = np.array((node.target.pos.point.x, node.target.pos.point.y, node.target.pos.point.z))
		nodeD = np.linalg.norm(startA - nodeA)
		if nodeD < targetNodeD and node.target.surface == start.surface:
			targetNode = node.target

	sEdge = Edge(start,minS,mindS,current_pose.transform.rotation)
	verticeGraph[start.id] = [sEdge]

	# find path from start to end, return list of node ids and dictionary of path edges using node ids as keys
	pathNodes, pathEdges = findPath.aStar(verticeGraph, start.id, targetNode.id)

	# visualize path
	path = Marker()
	# path.header.frame_id = "robot_odom_frame"
	path.header.frame_id = "camera_odom_frame"
	path.header.stamp = rospy.get_rostime()
	path.type = path.LINE_LIST
	path.action = path.ADD
	id += 1
	path.id = id
	path.scale.x = 0.01
	path.pose.orientation.x = 0.0
	path.pose.orientation.y = 0.0
	path.pose.orientation.z = 0.0
	path.pose.orientation.w = 1.0
	path.color.r = 1
	path.color.g = 1
	path.color.b = 0
	path.color.a = 1

	pathPoints = Path()

	# find transformation from starting pose to the first waypoint
	st = findStartTransform(minS, start, tfBuffer)
	pathPoints.path.append(st)
	path.points.append(start.pos.point)
	path.points.append(minS.pos.point)

	end_frame_id = 0
	# create transforms from waypoint to waypoint
	for i in range(2,len(pathNodes)):
		path.points.append(pathEdges[pathNodes[i]].source.pos.point)
		path.points.append(pathEdges[pathNodes[i]].target.pos.point)
		t = TransformStamped()
		t.transform = Transform(Vector3(pathEdges[pathNodes[i]].target.pos.point.x, pathEdges[pathNodes[i]].target.pos.point.y, pathEdges[pathNodes[i]].target.pos.point.z), 
										pathEdges[pathNodes[i]].rotation)
		# t.header.frame_id = 'robot_odom_frame'		
		t.header.frame_id = 'camera_odom_frame'
		t.child_frame_id = str(i)
		pathPoints.path.append(t)
		end_frame_id = i+1

	# find transform from final node to the end goal
	et = findEndTransform(end, targetNode, tfBuffer, end_frame_id)
	pathPoints.path.append(et)
	path.points.append(targetNode.pos.point)
	path.points.append(end.pos.point)
	
	# publish transform array and visualizations
	broadcaster.sendTransform(pathPoints.path)
	path_pub.publish(pathPoints)
	vis_pub.publish(path)

def pathModel():
	rospy.init_node('model')
	vis_puba = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
	vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	rate = rospy.Rate(10)
	rospy.wait_for_message('/camera/odom/sample', Odometry, timeout=5)
	
	F = FindEdge()
	S = Surfaces()
	verticeArray=MarkerArray()
	global verticeGraph
	global points
	global id

	# broadcast frames of surface
	broadcaster.sendTransform([S.surfaceA.getFrame(Quaternion(0,0,0,1)), 
							   S.surfaceB.getFrame(Quaternion(0, -0.707, 0, 0.707)),
							   S.surfaceC.getFrame(Quaternion(0.707, 0, 0.707, 0)),
							   S.surfaceD.getFrame(Quaternion(0.5, -0.5, 0.5, 0.5))])

	#====Build Structure Model===
	# Create base surface model
	surface=Marker()
	# surface.header.frame_id = "robot_odom_frame"
	surface.header.frame_id = "camera_odom_frame"
	surface.header.stamp = rospy.get_rostime()
	surface.ns = "Namespace"
	surface.id = id
	surface.type = surface.CUBE
	surface.action = surface.ADD
	surface.scale.x = S.xDim
	surface.scale.y = S.yDim
	surface.scale.z = S.zDim
	surface.pose.position.x = S.xDim/2 + S.xOffset
	surface.pose.position.y = S.yDim/2 + S.yOffset
	surface.pose.position.z = S.zDim/2 + S.zOffset
	surface.pose.orientation.x = 0.0
	surface.pose.orientation.y = 0.0
	surface.pose.orientation.z = 0.0
	surface.pose.orientation.w = 1.0
	surface.color.r = 0.5
	surface.color.g = 0.5
	surface.color.b = 0.5
	surface.color.a = 1.0
	id += 1
	
	#===Create Vertices===
	row,col = (2, 3)
	
	for i in range(row+1):
		for j in range(col):
			p = Vertice(PointStamped(),S.surfaceA)
			p.frame_pos.header.frame_id = 'surfaceA'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceA', i, j)
			p.frame_pos.point.x = (S.xDim/row) * (i)
			p.frame_pos.point.y = (S.yDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0) and not (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0):
				p.edge = True
			elif (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0) and not (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

	row,col = (6, 3)
	for i in range(row):
		for j in range(1,col):
			p = Vertice(PointStamped(),S.surfaceB)
			p.frame_pos.header.frame_id = 'surfaceB'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceB', i, j)
			p.frame_pos.point.x = (S.zDim/row) * (i)
			p.frame_pos.point.y = (S.yDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),S.surfaceB)
				gp.frame_pos.header.frame_id = 'surfaceB'
				gp.frame_pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceBF', i, j)
				gp.ground = True
				gp.frame_pos.point.x = 0
				gp.frame_pos.point.y = (S.yDim/col) * (j)
				gp.frame_pos.point.z = (S.zDim/row)
				points.append(gp)

	for i in range(row):
		for j in range(1,col):
			p = Vertice(PointStamped(),S.surfaceC)
			p.frame_pos.header.frame_id = 'surfaceC'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceC', i, j)
			p.frame_pos.point.x = (S.zDim/row) * (i)
			p.frame_pos.point.y = (S.yDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == S.xDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == S.yDim or p.frame_pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),S.surfaceC)
				gp.frame_pos.header.frame_id = 'surfaceC'
				gp.frame_pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceCF', i, j)
				gp.ground = True
				gp.frame_pos.point.x = 0
				gp.frame_pos.point.y = (S.yDim/col) * (j)
				gp.frame_pos.point.z = (S.zDim/row)
				points.append(gp)

	row,col = (6, 2)
	for i in range(row+1):
		for j in range(col+1):
			p = Vertice(PointStamped(),S.surfaceD)
			p.frame_pos.header.frame_id = 'surfaceD'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceD', i, j)
			p.frame_pos.point.x = (S.zDim/row) * (i)
			p.frame_pos.point.y = (S.xDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == S.zDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == S.xDim or p.frame_pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),S.surfaceD)
				gp.frame_pos.header.frame_id = 'surfaceD'
				gp.frame_pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceDF', i, j)
				gp.ground = True
				gp.frame_pos.point.x = 0
				gp.frame_pos.point.y = (S.xDim/col) * (j)
				gp.frame_pos.point.z = (S.zDim/row)
				points.append(gp)
		
	# Transform point from surface frame to base frame and create marker
	for p in points:
		# p.pos = tfBuffer.transform(p.frame_pos, 'robot_odom_frame', rospy.Duration(10))
		p.pos = tfBuffer.transform(p.frame_pos, 'camera_odom_frame', rospy.Duration(10))
		vertice = Marker()
		# vertice.header.frame_id = "robot_odom_frame"
		vertice.header.frame_id = "camera_odom_frame"
		vertice.header.stamp = rospy.get_rostime()
		vertice.type = vertice.SPHERE
		vertice.action = vertice.ADD
		vertice.id = id
		id += 1
		vertice.scale.x = 0.01
		vertice.scale.y = 0.01
		vertice.scale.z = 0.01
		vertice.pose.position = p.pos.point
		vertice.pose.orientation.x = 0.0
		vertice.pose.orientation.y = 0.0
		vertice.pose.orientation.z = 0.0
		vertice.pose.orientation.w = 1.0
		vertice.color.r = 0
		vertice.color.g = 0
		vertice.color.b = 0
		vertice.color.a = 1
		# if p.edge:
		# 	vertice.color.r = 0.5
		# 	vertice.color.g = 0
		# 	vertice.color.b = 1
		# 	vertice.color.a = 1
		verticeArray.markers.append(vertice)
	
	#create graph	
	for j in points:
		vert=j.id
		edges = []
		for k in points:
			a = np.array((j.pos.point.x, j.pos.point.y, j.pos.point.z))
			b = np.array((k.pos.point.x, k.pos.point.y, k.pos.point.z))
			dist = np.linalg.norm(a-b)
			if (not (j.edge and k.edge) and not(j.ground and k.ground) and # ensure edge points cannot connect to edge points and ground points cannont connect to ground points
			  ((dist < 0.33 and not j.pos == k.pos and j.surface == k.surface and not (j.edge or k.edge or j.ground or k.ground)) or # establish connections on face of surface, excluding edge points and ground points
			   (dist < 0.245 and j.ground != k.ground and j.surface == k.surface) or # establish connections to ground points
			   (dist < 0.245 and j.edge != k.edge and j.surface == k.surface) or # establish connections to edge points
			   (dist < 0.245 and j.edge != k.edge and j.surface != k.surface and (j.surface in S.surfaces[k.surface.id])))): # establish connections between surfaces
				edge = F.getEdge(j,k,dist,tfBuffer)
				edges.append(edge)

		verticeGraph[vert] = edges

	# visualize connections between vertices	
	line = Marker()
	# line.header.frame_id = "robot_odom_frame"
	line.header.frame_id = "camera_odom_frame"
	line.header.stamp = rospy.get_rostime()
	line.type = vertice.LINE_LIST
	line.action = vertice.ADD
	line.id = id
	line.scale.x = 0.005
	line.pose.orientation.x = 0.0
	line.pose.orientation.y = 0.0
	line.pose.orientation.z = 0.0
	line.pose.orientation.w = 1.0
	line.color.r = 0
	line.color.g = 0
	line.color.b = 0
	line.color.a = 1
		
	for vert in verticeGraph:
		for nxtvert in verticeGraph[vert]:
			line.points.append(nxtvert.target.pos.point)
			line.points.append(nxtvert.source.pos.point)

	# publish markers and listen for new target
	rospy.sleep(2)
	while not rospy.is_shutdown():
		vis_pub.publish(surface)
		vis_puba.publish(verticeArray)
		vis_pub.publish(line)
		targetmsg = rospy.wait_for_message('Target', Point)
		findPath(targetmsg, tfBuffer)
		rospy.sleep(3)
		
if __name__ == '__main__':
    try:
        pathModel()
    except rospy.ROSInterruptException:
        pass