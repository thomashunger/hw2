#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from tf import TransformListener
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool

class MarkerBasics(object):
	def __init__(self):
		self.marker_objectpublisher= rospy.Publisher('/marker_basic', Marker, queue_size=1)
		self._pub_co = rospy.Publisher('/ur5/collision_object', CollisionObject, queue_size=100)	
		self._sub_co = rospy.Subscriber('/ur5/collision_object', CollisionObject, self.sub_callback_co) 		
		self._sub_attached = rospy.Subscriber('/attached', Bool, self.sub_callback_attached) 		
		self.sub_tag = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.sub_callback_tag)
		self.rate= rospy.Rate(10)
		self.objects = None
		self.tf = TransformListener()
		self.post = False
		self.co = CollisionObject()
		self.attached = False

	def frame_transformation(self, _from = '/tag_11', _to = '/camera_rgb_optical_frame'):
		try:
			(trans,rot) = self.tf.lookupTransform(_to, _from, rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			pass
		print('trans: ', trans)
		print('rot: ', rot)
		self.objects.pose.pose.pose.position.x = trans[0]
		self.objects.pose.pose.pose.position.y = trans[1]
		self.objects.pose.pose.pose.position.z = trans[2]+self.objects.size[0]
		self.objects.pose.pose.pose.orientation.x = rot[0]
		self.objects.pose.pose.pose.orientation.y = rot[1]
		self.objects.pose.pose.pose.orientation.z = rot[2]
		self.objects.pose.pose.pose.orientation.w = rot[3]

	def camera_to_world(self):
		for i in range(len(self.objects)):
			ps = PoseStamped()
			ps.header.stamp = rospy.Time(0)
			ps.header.frame_id = '/camera_rgb_optical_frame'
			ps.pose.position = self.objects[i].pose.pose.pose.position
			ps.pose.orientation = self.objects[i].pose.pose.pose.orientation
			ps = self.tf.transformPose('/world', ps)
			self.objects[i].pose.pose.pose.position = ps.pose.position
			self.objects[i].pose.pose.pose.orientation = ps.pose.orientation
			self.objects[i].pose.header.frame_id = '/world'

	def sub_callback_co(self, msg):
		self.co = msg

	def sub_callback_attached(self, msg):
		self.attached = msg

	def sub_callback_tag(self, msg):
		self.post = True
		if (msg.detections != []):
			self.objects = msg.detections
			for i in range(len(self.objects)):
				rospy.loginfo('I heard %s', self.objects[i].id[0])

	def make_box(self, name, header, pose, size, scale, actualize = '1'):
		 box_names = ['9', '10', '11', '12', '0', '1', '2', '3']
		 triangle_names = ['13', '14', '15', '6', '7', '8']
		 exagon_names = ['4', '5']

		 if (actualize == '1'):
			 self.co.operation = CollisionObject.APPEND
		 elif (actualize == '2'):
			 self.co.operation = CollisionObject.ADD
		 elif (actualize == '3'):
			self.co.operation = CollisionObject.REMOVE
		 self.co.id = name
		 print('co.id: ', self.co.id)
		 self.co.header = header
		 box = SolidPrimitive()
		 if name in box_names:
			 box.type = SolidPrimitive.BOX
		 elif name in triangle_names:
			 box.type = SolidPrimitive.CONE
		 elif name in exagon_names:
			 box.type = SolidPrimitive.CYLINDER
		 size = [(x*scale) for x in size]
		 box.dimensions = list(size)
		 self.co.primitives = [box]
		 self.co.primitive_poses = [pose.pose]
		 return self.co

	def start(self):
		while (self.objects == None):
			#wait till the first Tag detection
			pass
		old_len = 0
		while (not rospy.is_shutdown() and not self.attached):
			if (old_len != len(self.objects)):
				print (old_len, len(self.objects))
				actualize = '3' #Remove
				#removing all the previous CollisionObjects
				for i in range(len(self.objects)):
					co = self.make_box(name=str(self.objects[i].id[0]), header=self.objects[i].pose.header, pose=self.objects[i].pose.pose, size=(self.objects[i].size)*3, scale=2, actualize = actualize)
					self._pub_co.publish(co)
					old_len = len(self.objects)
				actualize = '1' #APPEND
			#Actualize CollisionObjects
			if self.post:
				print('bckajhdvkasjvshjaas')
				self.camera_to_world()
				for i in range(len(self.objects)):
					co = self.make_box(name=str(self.objects[i].id[0]), header=self.objects[i].pose.header, pose=self.objects[i].pose.pose, size=(self.objects[i].size)*3, scale=2, actualize = actualize)
					self._pub_co.publish(co)
			self.post = False
			actualize = '2' #ADD
			self.rate.sleep()

if __name__ == '__main__':
	rospy.init_node('marker_basic_node', anonymous = True)
	markerbasics_object = MarkerBasics()
	try:
		markerbasics_object.start()
	except rospy.ROSInterruptException:
		pass





