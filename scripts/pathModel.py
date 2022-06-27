#!/usr/bin/env python2
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3, Pose
from tf2_geometry_msgs import PointStamped, PoseStamped
import tf2_ros 
from climbing_robot.msg import Path
import numpy as np
from aStar import Star

											  # Euler Angles (roll,pitch,yaw) (degrees)
QUATERNIONS = [Quaternion(0,0,0,1), 						#(0,0,0)			
			   Quaternion(0,0,0.382,0.924),					#(0,0,45)
			   Quaternion(0,0,0.707,0.707),					#(0,0,90)
			   Quaternion(0,0,0.924,0.382),					#(0,0,135)	
			   Quaternion(0,0,1,0),							#(0,0,180)
			   Quaternion(0,0,-0.924,0.382),				#(0,0,-135)
			   Quaternion(0,0,-0.707,0.707),				#(0,0,-90)
			   Quaternion(0,0,-0.382,0.924),				#(0,0,-45)

			   Quaternion(0,-0.382,0,0.924),
			   Quaternion(-0.271,-0.271,0.653,0.653),
			   Quaternion(-0.382,0,0.924,0),
			   Quaternion(0.271,-0.271,-0.653,0),
			   Quaternion(0,0.382,0,0.924),					#(0,45,0)
			   Quaternion(0.271,0.271,0.653,0.653),			#(0,45,90)
			   Quaternion(0.382,0,0.924,0),					#(0,45,180)
			   Quaternion(-0.271,0.271,-0.635,0.635),		#(0,45,-90)
			   
			   Quaternion(0,0.707,0,0.707)]		

class Surface:
	def __init__(self, id, xMin=0, xMax=0, yMin=0, yMax=0, zMin=0, zMax=0, xDim=0, yDim=0):
		self.id = id
		self.xMin = xMin
		self.xMax = xMax
		self.yMin = yMin
		self.yMax = yMax
		self.zMin = zMin
		self.zMax = zMax
		self.xDim = xDim
		self.yDim = yDim
	
	def getFrame(self, frame_rotation):
		frame = TransformStamped()
		frame.header.stamp = rospy.Time.now()
		frame.header.frame_id = 'robot_odom_frame'
		frame.child_frame_id = self.id
		frame.transform.translation = Vector3(self.xMin, self.yMin, self.zMin)
		frame.transform.rotation = frame_rotation
		return frame

	def __str__(self):
		return self.id

class Vertice:
	def __init__(self, frame_pos, surface):
		self.id = ''
		self.frame_pos = frame_pos
		self.surface = surface
		self.edge = False
		self.ground = False
		self.pos = PointStamped()

	def __str__(self):
		return "({0}, {1})".format(self.pos, self.surface)

class Edge:
	def __init__(self,source,target,distance,rotation):
		self.source = source
		self.target = target
		self.distance = distance
		self.rotation = rotation

def getEdge(source,target,dist,tfBuffer):
	source_frame = source.surface.id
	target_frame = target.surface.id
	source_pos = source.frame_pos
	target_pos = target.frame_pos
	if target_frame != source_frame:
		target_pos = tfBuffer.transform(target_pos, source_frame, rospy.Duration(10))
		if np.linalg.norm(source_pos.point.x - target_pos.point.x) < 0.1:
			target_pos.point.x = source_pos.point.x
		if np.linalg.norm(source_pos.point.y - target_pos.point.y) < 0.1:
			target_pos.point.y = source_pos.point.y

	q = PoseStamped()
	q.header.frame_id = source_frame
	if source.edge and source.pos.point.z == 0:
		q.pose = Pose(source_pos.point,QUATERNIONS[12])
	elif source.edge and source_frame == target_frame:
		if source_pos.point.x == 0:
			q.pose = Pose(source_pos.point,QUATERNIONS[8])
		elif source_pos.point.y == 0:
			q.pose = Pose(source_pos.point,QUATERNIONS[9])
		elif source_pos.point.x == source.surface.xDim:
			q.pose = Pose(source_pos.point,QUATERNIONS[10])
		elif source_pos.point.y == source.surface.yDim:
			q.pose = Pose(source_pos.point,QUATERNIONS[11])
	elif source.edge and source_frame != target_frame:
		if source_pos.point.x == 0:
			q.pose = Pose(source_pos.point,QUATERNIONS[12])
		elif source_pos.point.y == 0:
			q.pose = Pose(source_pos.point,QUATERNIONS[13])
		elif source_pos.point.x == source.surface.xDim:
			q.pose = Pose(source_pos.point,QUATERNIONS[14])
		elif source_pos.point.y == source.surface.yDim:
			q.pose = Pose(source_pos.point,QUATERNIONS[15])
	else:
		if source_pos.point.x < target_pos.point.x and source_pos.point.y == target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[0])
		elif source_pos.point.x > target_pos.point.x and source_pos.point.y < target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[1])
		elif source_pos.point.x == target_pos.point.x and source_pos.point.y < target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[2])
		elif source_pos.point.x > target_pos.point.x and source_pos.point.y < target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[3])
		elif source_pos.point.x > target_pos.point.x and source_pos.point.y == target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[4])
		elif source_pos.point.x > target_pos.point.x and source_pos.point.y > target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[5])
		elif source_pos.point.x == target_pos.point.x and source_pos.point.y > target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[6])
		elif source_pos.point.x < target_pos.point.x and source_pos.point.y > target_pos.point.y:
			q.pose = Pose(source_pos.point,QUATERNIONS[7])
		elif source.ground:
			q.pose = Pose(source_pos.point,QUATERNIONS[16])

	q = tfBuffer.transform(q, 'robot_odom_frame', rospy.Duration(10))

	return Edge(source,target,dist,q.pose.orientation)

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
	path_pub = rospy.Publisher("generated_path", Path, queue_size=10)
	broadcaster = tf2_ros.StaticTransformBroadcaster()
	
	rate = rospy.Rate(10)

	surfaceA = Surface('surfaceA', xOffset, (xOffset + xDim), yOffset, (yOffset + yDim), (zOffset + zDim), (zOffset + zDim), xDim, yDim)
	surfaceB = Surface('surfaceB', xOffset, xOffset, yOffset, (yOffset + yDim), zOffset, (zOffset + zDim), zDim, yDim)
	surfaceC = Surface('surfaceC', (xOffset + xDim), (xOffset + xDim), (yOffset + yDim), yOffset, zOffset, (zOffset + zDim), zDim, yDim)
	surfaceD = Surface('surfaceD', (xOffset + xDim), xOffset, yOffset, yOffset, zOffset, (zOffset + zDim), zDim, xDim)
	surfaceF = Surface('surfaceF', 0, 5, 0, 5, 0, 5)
	broadcaster.sendTransform([surfaceA.getFrame(Quaternion(0,0,0,1)), 
							   surfaceB.getFrame(Quaternion(0, -0.707, 0, 0.707)),
							   surfaceC.getFrame(Quaternion(0.707, 0, 0.707, 0)),
							   surfaceD.getFrame(Quaternion(0.5, -0.5, 0.5, 0.5))])
	
	
	surfaces = {}
	surfaces[str(surfaceA)] = [surfaceB, surfaceC, surfaceD]
	surfaces[str(surfaceB)] = [surfaceA, surfaceD, surfaceF]
	surfaces[str(surfaceC)] = [surfaceA, surfaceD, surfaceF]
	surfaces[str(surfaceD)] = [surfaceA, surfaceB, surfaceC, surfaceF]
	surfaces[str(surfaceF)] = [surfaceB, surfaceC, surfaceD]
	
	id = 0
	#====Build Structure Model===
	surface=Marker()
	surface.header.frame_id = "robot_odom_frame"
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
			p.frame_pos.header.frame_id = 'surfaceA'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceA', i, j)
			p.frame_pos.point.x = (xDim/row) * (i)
			p.frame_pos.point.y = (yDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == xDim or p.frame_pos.point.x == 0) and not (p.frame_pos.point.y == yDim or p.frame_pos.point.y == 0):
				p.edge = True
			elif (p.frame_pos.point.y == yDim or p.frame_pos.point.y == 0) and not (p.frame_pos.point.x == xDim or p.frame_pos.point.x == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

	row,col = (6, 3)
	for i in range(row):
		for j in range(1,col):
			p = Vertice(PointStamped(),surfaceB)
			p.frame_pos.header.frame_id = 'surfaceB'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceB', i, j)
			p.frame_pos.point.x = (zDim/row) * (i)
			p.frame_pos.point.y = (yDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == xDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == yDim or p.frame_pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),surfaceB)
				gp.frame_pos.header.frame_id = 'surfaceB'
				gp.frame_pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceBF', i, j)
				gp.ground = True
				gp.frame_pos.point.x = 0
				gp.frame_pos.point.y = (yDim/col) * (j)
				gp.frame_pos.point.z = (zDim/row)
				points.append(gp)

	for i in range(row):
		for j in range(1,col):
			p = Vertice(PointStamped(),surfaceC)
			p.frame_pos.header.frame_id = 'surfaceC'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceC', i, j)
			p.frame_pos.point.x = (zDim/row) * (i)
			p.frame_pos.point.y = (yDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == xDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == yDim or p.frame_pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),surfaceC)
				gp.frame_pos.header.frame_id = 'surfaceC'
				gp.frame_pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceCF', i, j)
				gp.ground = True
				gp.frame_pos.point.x = 0
				gp.frame_pos.point.y = (yDim/col) * (j)
				gp.frame_pos.point.z = (zDim/row)
				points.append(gp)

	row,col = (6, 2)
	for i in range(row+1):
		for j in range(col+1):
			p = Vertice(PointStamped(),surfaceD)
			p.frame_pos.header.frame_id = 'surfaceD'
			p.frame_pos.header.stamp = rospy.Time(0)
			p.id = "%s_%d_%d" %('surfaceD', i, j)
			p.frame_pos.point.x = (zDim/row) * (i)
			p.frame_pos.point.y = (xDim/col) * (j)
			p.frame_pos.point.z = 0
			if (p.frame_pos.point.x == zDim or p.frame_pos.point.x == 0) != (p.frame_pos.point.y == xDim or p.frame_pos.point.y == 0):
				p.edge = True
			if not ((i == row and j == col) or (i == 0 and j == col) or (i == row and j ==0) or (i == 0 and j == 0)):
				points.append(p)

			if i == 0 and not (j == 0 or j == col):
				gp = Vertice(PointStamped(),surfaceD)
				gp.frame_pos.header.frame_id = 'surfaceD'
				gp.frame_pos.header.stamp = rospy.Time(0)
				gp.id = "%s_%d_%d" %('surfaceDF', i, j)
				gp.ground = True
				gp.frame_pos.point.x = 0
				gp.frame_pos.point.y = (xDim/col) * (j)
				gp.frame_pos.point.z = (zDim/row)
				points.append(gp)
		
	#create graph	
	tfBuffer = tf2_ros.Buffer()
	listener = tf2_ros.TransformListener(tfBuffer)
	verticeGraph = {}

	for p in points:
		p.pos = tfBuffer.transform(p.frame_pos, 'robot_odom_frame', rospy.Duration(10))
		vertice = Marker()
		vertice.header.frame_id = "robot_odom_frame"
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
			if (not (j.edge and k.edge) and not(j.ground and k.ground) and ((dist < 0.33 and not j.pos == k.pos and j.surface == k.surface and not (j.edge or k.edge or j.ground or k.ground)) or 
			   (dist < 0.245 and j.ground != k.ground and j.surface == k.surface) or
			   (dist < 0.245 and j.edge != k.edge and j.surface == k.surface) or
			   (dist < 0.245 and j.edge != k.edge and j.surface != k.surface and (j.surface in surfaces[k.surface.id])))):
				edge = getEdge(j,k,dist,tfBuffer)
				edges.append(edge)

		verticeGraph[vert] = edges

	#plot graph edges		
	line = Marker()
	line.header.frame_id = "robot_odom_frame"
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

	current_pose = tfBuffer.lookup_transform('robot_odom_frame', 'robot_pose_frame', rospy.Time())

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
	


	sEdge = Edge(start,minS,mindS,current_pose.transform.rotation)
	verticeGraph[start.id] = [sEdge]
	
	pathNodes, pathEdges = findPath.aStar(verticeGraph, start.id, targetNode.id)
	
	path = Marker()
	path.header.frame_id = "robot_odom_frame"
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

	pathPoints = Path()
	for i in range(1,len(pathNodes)):
		path.points.append(pathEdges[pathNodes[i]].source.pos.point)
		path.points.append(pathEdges[pathNodes[i]].target.pos.point)
		if i != 1:
			t = TransformStamped()
			t.transform = Transform(pathEdges[pathNodes[i]].source.pos.point, pathEdges[pathNodes[i]].rotation)
			t.header.frame_id = 'robot_odom_frame'
			pathPoints.path.append(t)

	ed = endA - targetNodeA
	eRot = np.arctan2(ed[1],ed[0])
	ez = np.sin(eRot/2)
	ew = np.cos(eRot/2)
	e = PoseStamped()
	e.pose = Pose(end.pos.point, Quaternion(0,0,ez,ew))
	e.header.frame_id = targetNode.surface.id
	e = tfBuffer.transform(e, 'robot_odom_frame', rospy.Duration(10))
	et = TransformStamped()
	et.transform.translation = end.pos.point
	et.transform.rotation = e.pose.orientation
	et.header.frame_id = 'robot_odom_frame'
	et.child_frame_id = 'end'
	pathPoints.path.append(et)
	path.points.append(targetNode.pos.point)
	path.points.append(end.pos.point)
		
	#publish markers
	while not rospy.is_shutdown():
		vis_pub.publish(surface)
		vis_puba.publish(verticeArray)
		vis_pub.publish(line)
		vis_pub.publish(path)
		path_pub.publish(pathPoints)
		rate.sleep()

if __name__ == '__main__':
    try:
        pathModel()
    except rospy.ROSInterruptException:
        pass