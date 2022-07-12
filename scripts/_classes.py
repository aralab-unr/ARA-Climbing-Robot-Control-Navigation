#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Pose
from tf2_geometry_msgs import PointStamped, PoseStamped
import numpy as np

class Surface:
	def __init__(self, id='robot_odom_frame', xMin=0, xMax=0, yMin=0, yMax=0, zMin=0, zMax=0, xDim=0, yDim=0):
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
		frame.transform.translation = Point(self.xMin, self.yMin, self.zMin)
		frame.transform.rotation = frame_rotation
		return frame

	def __str__(self):
		return self.id

class Surfaces:
    xDim = 0.381
    yDim = 0.722
    zDim = 1.317
    xOffset = 0.5
    yOffset = -0.361
    zOffset = 0
    
    surfaceA = Surface('surfaceA', xOffset, (xOffset + xDim), yOffset, (yOffset + yDim), (zOffset + zDim), (zOffset + zDim), xDim, yDim)
    surfaceB = Surface('surfaceB', xOffset, xOffset, yOffset, (yOffset + yDim), zOffset, (zOffset + zDim), zDim, yDim)
    surfaceC = Surface('surfaceC', (xOffset + xDim), (xOffset + xDim), (yOffset + yDim), yOffset, zOffset, (zOffset + zDim), zDim, yDim)
    surfaceD = Surface('surfaceD', (xOffset + xDim), xOffset, yOffset, yOffset, zOffset, (zOffset + zDim), zDim, xDim)
    surfaceF = Surface('surfaceF', 0, 5, 0, 5, 0, 5)
    surfaces = {}

    surfaces[str(surfaceA)] = [surfaceB, surfaceC, surfaceD]
    surfaces[str(surfaceB)] = [surfaceA, surfaceD, surfaceF]
    surfaces[str(surfaceC)] = [surfaceA, surfaceD, surfaceF]
    surfaces[str(surfaceD)] = [surfaceA, surfaceB, surfaceC, surfaceF]
    surfaces[str(surfaceF)] = [surfaceB, surfaceC, surfaceD]

class Vertice:
	def __init__(self, frame_pos = PointStamped(), surface = Surface()):
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
   


class FindEdge:
											      # Euler Angles (roll,pitch,yaw) (degrees)
    QUATERNIONS = [ Quaternion(0,0,0,1), 						    #(0,0,0)			
                    Quaternion(0,0,0.382,0.924),					#(0,0,45)
                    Quaternion(0,0,0.707,0.707),					#(0,0,90)
                    Quaternion(0,0,0.924,0.382),					#(0,0,135)	
                    Quaternion(0,0,1,0),							#(0,0,180)
                    Quaternion(0,0,-0.924,0.382),				    #(0,0,-135)
                    Quaternion(0,0,-0.707,0.707),				    #(0,0,-90)
                    Quaternion(0,0,-0.382,0.924),				    #(0,0,-45)

                    Quaternion(0,0.382,0,0.924),					#(0,45,0)
                    Quaternion(0.271,0.271,0.653,0.653),			#(0,45,90)
                    Quaternion(0.382,0,0.924,0),					#(0,45,180)
                    Quaternion(-0.271,0.271,-0.635,0.635)]		    #(0,45,-90)		

    def getEdge(self,source,target,dist,tfBuffer):
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

        if source.ground:
            q.pose = Pose(target_pos.point,self.QUATERNIONS[8])
        elif target.edge:
            if np.linalg.norm(target_pos.point.x - source.surface.xDim) < 0.1:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[8])
            elif np.linalg.norm(target_pos.point.y - source.surface.yDim) < 0.1:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[9])
            elif np.linalg.norm(target_pos.point.x - 0) < 0.1:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[10])
            elif np.linalg.norm(target_pos.point.y - 0) < 0.1:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[11])
        else:
            if source_pos.point.x < target_pos.point.x and source_pos.point.y == target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[0])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y < target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[1])
            elif source_pos.point.x == target_pos.point.x and source_pos.point.y < target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[2])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y < target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[3])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y == target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[4])
            elif source_pos.point.x > target_pos.point.x and source_pos.point.y > target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[5])
            elif source_pos.point.x == target_pos.point.x and source_pos.point.y > target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[6])
            elif source_pos.point.x < target_pos.point.x and source_pos.point.y > target_pos.point.y:
                q.pose = Pose(target_pos.point,self.QUATERNIONS[7])

        q = tfBuffer.transform(q, 'robot_odom_frame', rospy.Duration(10))
        return Edge(source,target,dist,q.pose.orientation)