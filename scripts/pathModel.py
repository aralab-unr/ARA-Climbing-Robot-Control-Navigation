#!/usr/bin/env python2
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Point, PoseArray, Quaternion, QuaternionStamped, TransformStamped, Vector3, Transform
from tf2_geometry_msgs import PointStamped, QuaternionStamped
import tf2_ros
from climbing_robot.msg import Path
import numpy as np
from aStar import Star

			   #Quaternions                                                           	    Euler Angles (roll,pitch,yaw) (degrees)
QUATERNIONS = [QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,0,1)), 						#(0,0,0)			
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,0.382,0.924)),				#(0,0,45)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,0.707,0.707)),				#(0,0,90)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,0.924,0.382)),				#(0,0,135)	
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,1,0)),						#(0,0,180)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,-0.924,0.382)),				#(0,0,-135)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,-0.707,0.707)),				#(0,0,-90)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0,-0.382,0.924)),				#(0,0,-45)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0,0.382,0,0.924)),				#(0,45,0)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0.271,0.271,0.653,0.653)),		#(0,45,90)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(0.382,0,0.924,0)),				#(0,45,180)
			   QuaternionStamped(Header('camera_odom_frame'),Quaternion(-0.271,0.271,-0.635,0.635))]	#(0,45,-90)

class Surface:
	def __init__(self, id, xMin=0, xMax=0, yMin=0, yMax=0, zMin=0, zMax=0, xDim=0, yDim=0):
		self.id = id
		self.xMin = xMin
		self.xMax = xMax
		self.yMin = yMin
		self.yMax = yMax
		self.zMin = zMin
		self.zMax = zMax
	
	def getFrame(self, frame_id, frame_rotation):
		frame = TransformStamped()
		frame.header.stamp = rospy.Time.now()
		frame.header.frame_id = 'camera_odom_frame'
		frame.child_frame_id = frame_id
		frame.transform.translation = Vector3(self.xMin, self.yMin, self.zMin)
		frame.transform.rotation = frame_rotation
		return frame

	def __str__(self):
		return self.id

class Vertice:
	def __init__(self, pos, surface):
		self.id = ''
		self.pos = pos
		self.surface = surface
		self.edge = False

	def __str__(self):
		return "({0}, {1})".format(self.pos, self.surface)

class Edge:
	def __init__(self,source,target,distance,rotation):
		self.source = source
		self.target = target
		self.distance = distance
		self.rotation = rotation

def getEdge(source,target,dist):
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	source_frame = str("%s_%s" %(source.surface.id, 'frame'))
	target_frame = str("%s_%s" %(target.surface.id, 'frame'))
	source_pos = tfBuffer.transform(source.pos, source_frame, rospy.Duration(10))
	target_pos = tfBuffer.transform(target.pos, target_frame, rospy.Duration(10))
	q = Quaternion()
	if source_frame == target_frame:
		if source_pos.point.x > target_pos.point.x and source.pos_point.y == target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[0], source_frame, rospy.Duration(10))
		elif source_pos.point.x > target_pos.point.x and source.pos_point.y > target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[1], source_frame, rospy.Duration(10))
		elif source_pos.point.x == target_pos.point.x and source.pos_point.y > target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[2], source_frame, rospy.Duration(10))
		elif source_pos.point.x < target_pos.point.x and source.pos_point.y > target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[3], source_frame, rospy.Duration(10))
		elif source_pos.point.x < target_pos.point.x and source.pos_point.y == target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[4], source_frame, rospy.Duration(10))
		elif source_pos.point.x > target_pos.point.x and source.pos_point.y < target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[5], source_frame, rospy.Duration(10))
		elif source_pos.point.x == target_pos.point.x and source.pos_point.y < target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[6], source_frame, rospy.Duration(10))
		elif source_pos.point.x > target_pos.point.x and source.pos_point.y < target_pos.point.y:
			q = tfBuffer.transform(QUATERNIONS[7], source_frame, rospy.Duration(10))
	else:
		if source_pos.point.x == source.surface.xDim:
			q = tfBuffer.transform(QUATERNIONS[8], source_frame, rospy.Duration(10))
		elif source_pos.point.y == source.surface.yDim:
			q = tfBuffer.transform(QUATERNIONS[9], source_frame, rospy.Duration(10))
		elif source_pos.point.x == 0:
			q = tfBuffer.transform(QUATERNIONS[10], source_frame, rospy.Duration(10))
		elif source_pos.point.y == 0:
			q = tfBuffer.transform(QUATERNIONS[11], source_frame, rospy.Duration(10))
	return Edge(source,target,dist,q)

def pathModel():
	xDim = 0.381
	yDim = 0.722
	zDim = 1.317
	xOffset = 0.495
	yOffset = 0
	zOffset = 0
	
	rospy.init_node('model')
	vis_puba = rospy.Publisher("visualization_marker_array", MarkerArray, queue_size=10)
	vis_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
	path_pub = rospy.Publisher("generated_path", PoseArray, queue_size=10)
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	
	rate = rospy.Rate(10)

	surfaceA = Surface('surfaceA', xOffset, (xOffset + xDim), yOffset, (yOffset + yDim), (zOffset + zDim), (zOffset + zDim), xDim, yDim)
	surfaceB = Surface('surfaceB', xOffset, xOffset, yOffset, (yOffset + yDim), zOffset, (zOffset + zDim), zDim, yDim)
	surfaceC = Surface('surfaceC', (xOffset + xDim), (xOffset + xDim), (yOffset + yDim), yOffset, zOffset, (zOffset + zDim), zDim, yDim)
	surfaceD = Surface('surfaceD', (xOffset + xDim), xOffset, yOffset, yOffset, zOffset, (zOffset + zDim), zDim, xDim)
	surfaceF = Surface('surfaceF', 0, 5, 0, 5, 0, 5)
	broadcaster.sendTransform([surfaceA.getFrame('surfaceA_frame', Quaternion(0,0,0,1)), 
							   surfaceB.getFrame('surfaceB_frame', Quaternion(0, -0.707, 0, 0.707)),
							   surfaceC.getFrame('surfaceC_frame', Quaternion(0.707, 0, 0.707, 0)),
							   surfaceD.getFrame('surfaceD_frame', Quaternion(0.5, -0.5, 0.5, 0.5)),
							   surfaceF.getFrame('surfaceF_frame', Quaternion(0,0,0,1))])
	
	
	surfaces = {}
	surfaces[str(surfaceA)] = [surfaceB, surfaceC, surfaceD]
	surfaces[str(surfaceB)] = [surfaceA, surfaceD, surfaceF]
	surfaces[str(surfaceC)] = [surfaceA, surfaceD, surfaceF]
	surfaces[str(surfaceD)] = [surfaceA, surfaceB, surfaceC, surfaceF]
	surfaces[str(surfaceF)] = [surfaceB, surfaceC, surfaceD]
	
	id = 0
	#====Build Structure Model===
	surface=Marker()
	surface.header.frame_id = "camera_odom_frame"
	surface.header.stamp = rospy.get_rostime()
	surface.ns = "Namespace"
	surface.id = id
	surface.type = surface.CUBE
	surface.action = surface.ADD
	surface.scale.x = xDim
	surface.scale.y = yDim
	surface.scale.z = zDim
	surface.pose.position.x = xDim/2 + xOffset
	surface.pose.position.y = yDim/2 + yOffset
	surface.pose.position.z = zDim/2 + zOffset
	surface.pose.orientation.x = 0.0
	surface.pose.orientation.y = 0.0
	surface.pose.orientation.z = 0.0
	surface.pose.orientation.w = 1.0
	surface.color.r = 1.0
	surface.color.g = 1.0
	surface.color.b = 1.0
	surface.color.a = 1.0
	id += 1
	
	#===Create Vertices===
	verticeArray=MarkerArray()
	points = []
	row,col = (2, 3)
	
	for i in range(row+1):
		for j in range(col):
			p = Vertice(PointStamped(),surfaceA)
			p.pos.header.frame_id = 'surfaceA_frame'
			p.pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceA', i, j)
			p.pos.point.x = (xDim/row) * (i)
			p.pos.point.y = (yDim/col) * (j)
			p.pos.point.z = 0
			if (p.pos.point.x == xDim or p.pos.point.x == 0) and not (p.pos.point.y == yDim or p.pos.point.y == 0):
				p.edge = True
			elif (p.pos.point.y == yDim or p.pos.point.y == 0) and not (p.pos.point.x == xDim or p.pos.point.x == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

	row,col = (6, 3)
	for i in range(row):
		for j in range(1,col):
			p = Vertice(PointStamped(),surfaceB)
			p.pos.header.frame_id = 'surfaceB_frame'
			p.pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceB', i, j)
			p.pos.point.x = (zDim/row) * (i)
			p.pos.point.y = (yDim/col) * (j)
			p.pos.point.z = 0
			if (p.pos.point.x == xDim or p.pos.point.x == 0) != (p.pos.point.y == yDim or p.pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),surfaceF)
				gp.pos.header.frame_id = 'surfaceB_frame'
				gp.pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceBF', i, j)
				gp.pos.point.x = 0
				gp.pos.point.y = (yDim/col) * (j)
				gp.pos.point.z = (zDim/row)
				points.append(gp)

	for i in range(row):
		for j in range(1,col):
			p = Vertice(PointStamped(),surfaceC)
			p.pos.header.frame_id = 'surfaceC_frame'
			p.pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceC', i, j)
			p.pos.point.x = (zDim/row) * (i)
			p.pos.point.y = (yDim/col) * (j)
			p.pos.point.z = 0
			if (p.pos.point.x == xDim or p.pos.point.x == 0) != (p.pos.point.y == yDim or p.pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),surfaceF)
				gp.pos.header.frame_id = 'surfaceC_frame'
				gp.pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceCF', i, j)
				gp.pos.point.x = 0
				gp.pos.point.y = (yDim/col) * (j)
				gp.pos.point.z = (zDim/row)
				points.append(gp)

	row,col = (6, 2)
	for i in range(row+1):
		for j in range(col+1):
			p = Vertice(PointStamped(),surfaceD)
			p.pos.header.frame_id = 'surfaceD_frame'
			p.pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceD', i, j)
			p.pos.point.x = (zDim/row) * (i)
			p.pos.point.y = (xDim/col) * (j)
			p.pos.point.z = 0
			if (p.pos.point.x == zDim or p.pos.point.x == 0) != (p.pos.point.y == xDim or p.pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),surfaceF)
				gp.pos.header.frame_id = 'surfaceD_frame'
				gp.pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceDF', i, j)
				gp.pos.point.x = 0
				gp.pos.point.y = (xDim/col) * (j)
				gp.pos.point.z = (zDim/row)
				points.append(gp)
		
	#create graph	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	verticeGraph = {}

	for p in points:
		p.pos = tfBuffer.transform(p.pos, 'camera_odom_frame', rospy.Duration(10))
		vertice = Marker()
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
		vertice.color.b = 1
		vertice.color.a = 1
		if p.edge:
			vertice.color.r = 0.5
			vertice.color.g = 0
			vertice.color.b = 1
			vertice.color.a = 1
		verticeArray.markers.append(vertice)
	
	for j in points:
		vert=j.id
		edges = []
		for k in points:
			
			a = np.array((j.pos.point.x, j.pos.point.y, j.pos.point.z))
			b = np.array((k.pos.point.x, k.pos.point.y, k.pos.point.z))
			dist = np.linalg.norm(a-b)
			if (not (j.edge and k.edge) and ((dist < 0.33 and not j.pos == k.pos and j.surface == k.surface and j.surface != surfaceF and k.surface != surfaceF and not (j.edge or k.edge)) or 
			   (dist < 0.245 and j.edge != k.edge and j.surface != k.surface and (j.surface in surfaces[str(k.surface)])) or
			   (dist < 0.245 and j.edge != k.edge and j.surface == k.surface))):
				edge = getEdge(j,k,dist)
				edges.append(edge)
		verticeGraph[vert] = edges

	#plot graph edges		
	line = Marker()
	line.header.frame_id = "camera_odom_frame"
	line.header.stamp = rospy.get_rostime()
	line.type = vertice.LINE_LIST
	line.action = vertice.ADD
	line.id = id
	line.scale.x = 0.001
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

	current_pose = tfBuffer.lookup_transform('camera_odom_frame', 'camera_pose_frame', rospy.Time())

	findPath = Star()
	start = Vertice(PointStamped(), surfaceF)
	start.pos.point = Point(current_pose.transform.translation.x, current_pose.transform.translation.y, current_pose.transform.translation.z)
	start.id = 'start'
	end = Vertice(PointStamped(), surfaceF)
	end.pos.point = Point(.7,.7,1.317)
	end.id = 'end'
	
	startA = np.array((start.pos.point.x, start.pos.point.y, start.pos.point.z))
	endA = np.array((end.pos.point.x, end.pos.point.y, end.pos.point.z))
	mindS = 500
	mindE = 500
	minS = Vertice(Point(500,500,500), surfaceA)
	minE = Vertice(Point(500,500,500), surfaceA)

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
	
	targetNode = minE
	for node in verticeGraph[minE.id]:
		targetNodeA = np.array((targetNode.pos.point.x, targetNode.pos.point.y, targetNode.pos.point.z))
		targetNodeD = np.linalg.norm(startA - targetNodeA)
		nodeA = np.array((node.target.pos.point.x, node.target.pos.point.y, node.target.pos.point.z))
		nodeD = np.linalg.norm(startA - nodeA)
		if nodeD < targetNodeD and node.target.surface == start.surface:
			targetNode = node.target
	
	edge = Edge(start,minS,mindS,Quaternion())
	verticeGraph[start.id] = [edge]
	edge = Edge(minS, start, mindS, Quaternion())
	verticeGraph[minS.id].append(edge)
	edge = Edge(end,minE,mindE,Quaternion())
	verticeGraph[end.id] = [edge]
	edge = Edge(minE, end, mindE, Quaternion())
	verticeGraph[minE.id].append(edge)

	pathNodes, pathEdges = findPath.aStar(verticeGraph, start.id, end.id)
	
	path = Marker()
	path.header.frame_id = "camera_odom_frame"
	path.header.stamp = rospy.get_rostime()
	path.type = vertice.LINE_LIST
	path.action = vertice.ADD
	id += 1
	path.id = id
	path.scale.x = 0.01
	path.pose.orientation.x = 0.0
	path.pose.orientation.y = 0.0
	path.pose.orientation.z = 0.0
	path.pose.orientation.w = 1.0
	path.color.r = 0
	path.color.g = 1
	path.color.b = 0
	path.color.a = 1

	path.points.append(end.pos.point)
	path.points.append(targetNode.pos.point)

	pathPoints = PoseArray()
	pathPoints.poses.append(Pose(targetNode.pos, Quaternion()))
	for i in range(len(pathNodes)-1):
		path.points.append(pathEdges[pathNodes[i]][0].pos.point)
		path.points.append(pathEdges[pathNodes[i]][1].pos.point)
		print(pathNodes[i])
		# pathPoints.poses.append(Pose(pathEdges[pathNodes[i]][1].pos, Quaternion()))
	path.points.append(pathEdges[pathNodes[0]][0].pos.point)
	path.points.append(targetNode.pos.point)
	# pathPoints.poses.append(Pose(pathEdges[pathNodes[0]][0].pos, Quaternion()))
	# pathPoints.poses.append(Pose(end, Quaternion()))
		
	#publish markers
	while not rospy.is_shutdown():
		vis_pub.publish(surface)
		vis_puba.publish(verticeArray)
		vis_pub.publish(line)
		vis_pub.publish(path)
		# path_pub.publish(pathPoints)
		rate.sleep()

if __name__ == '__main__':
    try:
        pathModel()
    except rospy.ROSInterruptException:
        pass