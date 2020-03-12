#!/usr/bin/env python

import sys
import os
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import warnings
from math import pi
from moveit_commander.conversions import pose_to_list
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from tf import TransformListener
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class Manipulator(object):
	"""Manipulator"""
	def __init__(self):
		super(Manipulator, self).__init__()
		rospy.init_node('manipulator')
		moveit_commander.roscpp_initialize(sys.argv)
		warnings.filterwarnings("ignore")
		with warnings.catch_warnings():
			warnings.simplefilter("ignore")
			warnings.warn("deprecated", DeprecationWarning)

		robot = moveit_commander.RobotCommander(robot_description="/ur5/robot_description", ns='/ur5')
		scene = moveit_commander.PlanningSceneInterface()
		group_name = "manipulator"
		move_group = moveit_commander.MoveGroupCommander(group_name, robot_description="/ur5/robot_description", ns='/ur5')

		self._sub_co = rospy.Subscriber('/ur5/collision_object', CollisionObject, self.sub_callback_co)
		self._pub_co = rospy.Publisher('/ur5/collision_object', CollisionObject, queue_size=100)
		self._pub_aco = rospy.Publisher('/ur5/attached_collision_object', AttachedCollisionObject, queue_size=100)
		self._pub_attached = rospy.Publisher('/attached', Bool, queue_size=1)
		self.sub_tag = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.sub_callback_tag)

		self.robot = robot
		self.scene = scene
		self.move_group = move_group
		self.planning_frame = move_group.get_planning_frame()
		self.eef_link = move_group.get_end_effector_link()
		self.group_names = robot.get_group_names()
		self.co = CollisionObject()
		self.co_list = []
		self.tf = TransformListener()
		self.object_coordinates = None


		print "============ Planning frame: %s" % self.planning_frame

		print "============ End effector link: %s" % self.eef_link

		print "============ Available Planning Groups:", robot.get_group_names()

		print "============ Printing robot state"
		print robot.get_current_state()
		print ""

		print "============ Printing robot framework"
		print robot.get_planning_frame()
		print ""

		print "============ Printing objects in the scene"
		print scene.get_known_object_names()
		print ""
		print('move_group.get_current_joint_values(): ', move_group.get_current_joint_values())


################################################################################################
################################################################################################

	def sub_callback_co(self, msg):
		self.co = msg
		for y in self.co_list:
			if (msg.id == y.id):
				if (not(all_close(msg.primitive_poses[0], y.primitive_poses[0], 0.1))):
					y.primitive_poses = msg.primitive_poses
					rospy.loginfo('CollisionObject %s changed position', self.co.id)
				
		if (self.co_list == [] or not(msg.id in [x.id for x in self.co_list])):
			self.co_list.append(msg)
			rospy.loginfo('I heard a CollisionObject %s', self.co.id)
			rospy.loginfo('Pose of CollisionObject %s', self.co.primitive_poses)

	def sub_callback_tag(self, msg):
		if (msg.detections != []):
			self.object_coordinates = msg.detections[0]
			#rospy.loginfo('I heard a Tag %s', self.object_coordinates.id[0])
			#rospy.loginfo('I heard a Tag %s', self.object_coordinates.pose.pose.pose)

	def __submit(self, collision_object, attach=False):
		     print('you are in __submit')
		     if attach:
			 print('one')
			 self._pub_attached.publish(True)
		         self._pub_aco.publish(collision_object)
		     else:
			 print('two')
		         self._pub_co.publish(collision_object)

	def attach(self, selected_id_index = 0):
		grasping_group = 'end_effector'
		touch_links = self.robot.get_link_names(group=grasping_group)
		aco = AttachedCollisionObject()
		aco.object = self.co_list[selected_id_index]
		aco.link_name = 'ee_link'
		if len(touch_links) > 0:
		    aco.touch_links = touch_links
		else:
		    aco.touch_links = ['ee_link']
		self.__submit(aco, attach=True)

		return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=4)

	def detach(self, selected_id_index, timeout=4):
		grasping_group = 'end_effector'
		touch_links = self.robot.get_link_names(group=grasping_group)
		aco = AttachedCollisionObject()
		co = CollisionObject()
		co = self.co_list[selected_id_index]
		co.operation = CollisionObject.REMOVE
		aco.object = co
		aco.link_name = 'ee_link'
		if len(touch_links) > 0:
		    aco.touch_links = touch_links
		else:
		    aco.touch_links = ['ee_link']
		self.__submit(aco, attach=True)
		self.__submit(co, attach=False)

		return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


	def remove_box(self, timeout=4):
		self.scene.remove_world_object(self.co.id)

	# We wait for the planning scene to update.
		return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

################################################################################################
################################################################################################
	
	def go_to_joint_state(self):
		move_group = self.move_group

		joint_goal = move_group.get_current_joint_values()
		joint_goal[0] = -pi
		move_group.go(joint_goal, wait=True)

		# Calling ``stop()`` ensures that there is no residual movement
		move_group.stop()

		## END_SUB_TUTORIAL

		# For testing:
		current_joints = move_group.get_current_joint_values()
		return all_close(joint_goal, current_joints, 0.01)

	def plan_cartesian_path(self, m=1, scale=1):
		move_group = self.move_group
		if (m==1): #going near the object to be attached
		    waypoints = []

		    wpose = move_group.get_current_pose().pose
		    wpose.position.z -= scale * 0.15  # move down (-z)
		    waypoints.append(copy.deepcopy(wpose))

		    (plan, fraction) = move_group.compute_cartesian_path(
					               waypoints,   # waypoints to follow
					               0.01,        # eef_step
					               0.0)         # jump_threshold

		    move_group.execute(plan, wait=True)

		if (m==2): #going back with the object attached
		    waypoints = []

		    wpose = move_group.get_current_pose().pose
		    wpose.position.z += scale * 0.1  # move down (+z)
		    waypoints.append(copy.deepcopy(wpose))

		    (plan, fraction) = move_group.compute_cartesian_path(
					               waypoints,   # waypoints to follow
					               0.01,        # eef_step
					               0.0)         # jump_threshold

		    move_group.execute(plan, wait=True)

		if (m==3): #going back with the object attached
		    waypoints = []

		    wpose = move_group.get_current_pose().pose
		    wpose.position.z -= scale * 0.15  # Move down (-z)
		    wpose.position.x -= scale * 0.3  # and move right (-x)
		    waypoints.append(copy.deepcopy(wpose))

		    (plan, fraction) = move_group.compute_cartesian_path(
					               waypoints,   # waypoints to follow
					               0.01,        # eef_step
					               0.0)         # jump_threshold

		    move_group.execute(plan, wait=True)

	def go_to_pose_goal(self, m, selected_id_index):
		move_group = self.move_group

		if (m==1):
		    pose_goal = geometry_msgs.msg.Pose()
		    pose_goal.orientation.w = 0.7067
		    pose_goal.position.x = 0.1915
		    pose_goal.position.y = -0.0952
		    pose_goal.position.z = 1.7763
		if (m==2):
		    pose_goal = geometry_msgs.msg.Pose()
		    pose_goal.orientation.x = 0.5045
		    pose_goal.orientation.y = -0.4953
		    pose_goal.orientation.z = -0.4957
		    pose_goal.orientation.w = 0.5042
		    pose_goal.position.x = 0.1915
		    pose_goal.position.y = -0.0951
		    pose_goal.position.z = 1.7763
		if (m==3):
		    pose_goal = geometry_msgs.msg.Pose()
		    pose_goal.orientation.x = 0.7071
		    pose_goal.orientation.y = 0.0009
		    pose_goal.orientation.z = -0.7069
		    pose_goal.orientation.w = 0.0115
		    pose_goal.position.x = 0.1934
		    pose_goal.position.y = -0.2035
		    pose_goal.position.z = 1.6557
		if (m==4):
		    pose_goal = geometry_msgs.msg.Pose()
		    pose_goal.orientation.x = 0.7071
		    pose_goal.orientation.y = 0.0008
		    pose_goal.orientation.z = -0.7069
		    pose_goal.orientation.w = 0.0117
		    pose_goal.position.x = 0.1096
		    pose_goal.position.y = -0.4101
		    pose_goal.position.z = 1.4955
		if (m==5):
		    rospy.loginfo('pose of %s: %s ', self.co_list[selected_id_index].id, self.co_list[selected_id_index].primitive_poses)
		    #print('pose of %s: %s ', self.co_list[selected_id_index].id, self.co_list[selected_id_index].primitive_poses)
		    pose_goal = geometry_msgs.msg.Pose()
		    pose_goal.orientation.x = 0.7071
		    pose_goal.orientation.y = 0.0008
		    pose_goal.orientation.z = -0.7069
		    pose_goal.orientation.w = 0.0117
		    pose_goal.position.x = self.co_list[selected_id_index].primitive_poses[0].position.x
		    pose_goal.position.y = self.co_list[selected_id_index].primitive_poses[0].position.y
		    pose_goal.position.z = (self.co_list[selected_id_index].primitive_poses[0].position.z + 0.3)

		move_group.set_pose_target(pose_goal)
		plan = move_group.go(wait=True)
		move_group.stop()
		move_group.clear_pose_targets()

		current_pose = self.move_group.get_current_pose().pose
		return all_close(pose_goal, current_pose, 0.01)



	def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
		box_name = self.co.id
		scene = self.scene

		start = rospy.get_time()
		seconds = rospy.get_time()
		while (seconds - start < timeout) and not rospy.is_shutdown():
			# Test if the box is in attached objects
			attached_objects = scene.get_attached_objects([box_name])
			is_attached = len(attached_objects.keys()) > 0

			# Test if the box is in the scene.
			# Note that attaching the box will remove it from known_objects
			is_known = box_name in scene.get_known_object_names()

			# Test if we are in the expected state
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True

			# Sleep so that we give other threads time on the processor
			rospy.sleep(0.1)
			seconds = rospy.get_time()

		# If we exited the while loop without returning then we timed out
		return False

################################################################################################
################################################################################################


def main():
	try:
		print ""
		print "----------------------------------------------------------"
		print "Welcome to the UR5 Manipulator command Interface"
		print "----------------------------------------------------------"
		print "Press Ctrl-D to exit at any time"
		print ""
		print "============ Press `Enter` to start commanding the UR5 Manipulator ..."
		raw_input()
		ur5 = Manipulator()

		print "============ Press `Enter` to move to the home position ..."
		raw_input()
		#ur5.go_to_pose_goal(m=1)

		print('self.co_list: ', ur5.co_list)

		selected_id = ''
		id_list = [x.id for x in ur5.co_list]
		while selected_id not in id_list:
			print "============ Select one of the following Ids:..."
			print(id_list)
			selected_id = raw_input()
			if selected_id in id_list:
				break

		print "============ Press `Enter` to execute a movement using a pose goal ..."
		raw_input()
		#ur5.go_to_pose_goal(m=5, selected_id_index = id_list.index(selected_id))

		print "============ Press `Enter` to execute a cartesian movement ..."
		raw_input()
		#ur5.plan_cartesian_path(m=1)
		
		print "============ Press `Enter` to activate the Attachment ..."
		raw_input()
		#ur5.attach(selected_id_index = id_list.index(selected_id))
		os.system("rostopic pub --once /left_hand/command robotiq_3f_gripper_articulated_msgs/Robotiq3FGripperRobotOutput \"{rACT: 1, rMOD: 0, rGTO: 1, rATR: 0, rGLV: 0, rICF: 0, rICS: 0, rPRA: 250, rSPA: 200, rFRA: 200, rPRB: 0, rSPB: 0, rFRB: 0, rPRC: 0, rSPC: 0, rFRC: 0, rPRS: 0, rSPS: 0, rFRS: 0}\"")

		print "============ Press `Enter` to execute a cartesian movement ..."
		raw_input()
		#ur5.plan_cartesian_path(m=2)

		print "============ Press `Enter` to execute a base joint rotation of pi/2 ..."
		raw_input()
		#ur5.go_to_joint_state()

		print "============ Press `Enter` to execute a cartesian movement ..."
		raw_input()
		#ur5.plan_cartesian_path(m=3)

		print "============ Press `Enter` to desactivate the Attachment(deatach) ..."
		raw_input()
		#ur5.detach(selected_id_index = id_list.index(selected_id))
		os.system("rosservice call open_gripper command=\"2\"")


		print "============ Python tutorial demo complete!"
	except rospy.ROSInterruptException:
		return
	except KeyboardInterrupt:
		return

if __name__ == '__main__':
	main()
