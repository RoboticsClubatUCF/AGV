#!/usr/bin/env python

# general imports
from enum import Enum

# ROS system imports
import rospy
import actionlib
import smach, smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sens
import geometry_msgs.msg as geom
import move_base_msgs.msg as move_base
import ugv_msg.msg as ugv

class WaypointStatus(Enum):
    NOT_STARTED = -1
    IN_PROGRESS = 0
    COMPLETE = 1
    FAILED = 2

class Waypoint():

    def __init__(self, id, coords, frame_id, label=""):

        self.id = id
        self.frame_id = frame_id
        self.coords = coords
        self.label = label

        self.__status = WaypointStatus.NOT_STARTED

    @property
    def status(self):
        return self.__status
    
    @status.setter
    def status(self, status):
        self.__status = status

    def __repr__(self):
        return "\nWP {} \"{}\": \n\tframe_id: {}\n\tcoords: {}\n\tstatus: {}".format(self.id, self.label, self.frame_id, self.coords, self.__status.name)

    def as_goal(self):
        """
        Returns this Waypoint as a MoveBaseGoal, stamped with current ROS time
        """

        # convert waypoint to a move_base goal
        ag = MoveBaseGoal()
        # populate header (std_msgs/Header)
        ag.target_pose.header.stamp = rospy.Time.now()
        ag.target_pose.header.frame_id = self.frame_id
        # populate pose
        pose = geom.Pose()
        pose.position.x = self.coords[0]
        pose.position.y = self.coords[1]
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        ag.target_pose.pose = pose

        return ag
    
class WaypointList():

    def __init__(self, frame_id="map", namespace="waypoints"):

        self.waypoints = []
        self.frame_id = frame_id
        self.size = 0
        self.id = -1
        self.namespace = namespace

        self.current_WP = None
        self.progress = -1  # tracks which waypoint we're on in the list

    def get_next_waypoint(self):

        self.progress += 1
        if self.progress == len(self.waypoints):
            return None
        else:
            try:
                self.current_WP = self.waypoints[self.progress]
            except IndexError:
                rospy.logdebug("No more waypoints loaded.")
                return None
            return self.current_WP

    def get_next_goal(self):
        """
        Returns a move_base_msgs/MoveBaseGoal containing the next waypoint
        """
        wp = self.get_next_waypoint()
        if wp is not None:
            return wp.as_goal()
        else:
            return None

    def remove_waypoint(self, index=0):
        """
        Removes waypoint from list at given index (defaults to zero)
        """

        self.waypoints.pop(0)
        self.size -= 1
        
    def read_waypoint_params(self, namespace):
        """
        Read waypoints from the ROS Parameter server at the given namespace.
        Waypoints are assumed to be labeled wp_0, wp_1,...., wp_n
        """

        idx = 0
        while rospy.has_param('{}/wp_{}'.format(namespace, idx)):
            try:
                wp = rospy.get_param('{}/wp_{}'.format(namespace, idx))
                waypoint = Waypoint(id=idx, coords=wp["coords"], frame_id=wp["frame_id"], label=wp["label"])
                self.waypoints.append(waypoint)
                self.size += 1
                idx += 1
            except KeyError:
                return
        
    def reset(self):

        self.progress = -1

class Auto(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['standby', 'ESTOP', 'error'],
                                   output_keys=['auto_success'])

        self.AUTO = True
        self.ESTOP = False

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        # load waypoints from parameter server
        self.waypoints = WaypointList(frame_id='map', namespace='waypoints')
        self.waypoints.read_waypoint_params(namespace='waypoints')

        # define client for communication with move_base
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)

    def execute(self, userdata):

        self.state_pub.publish("AUTO")

        self.ESTOP = False  # reset ESTOP flag

        rospy.Subscriber('/choo_2/rc', ugv.RC, callback=self.rc_callback)

        # make sure we have connection to client server before continuing, timeout after 10s (return to standby)
        success = self.client.wait_for_server(timeout=rospy.Duration(10))
        if not success:
            rospy.logerr("Waiting for move_base action server timed out. Transition back to STANDBY")
            return 'standby'
        
        # send first waypoint to move_base to start navigation
        goal = self.waypoints.get_next_goal()
        rospy.logdebug("Starting AUTO state at WP {}{}".format(self.waypoints.current_WP.id, self.waypoints.current_WP))
        self.client.send_goal(goal)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # check flags
            if not self.AUTO:
                rospy.logdebug("AUTO switch disengaged. Cancelling current goal, resetting waypoints.")
                self.client.cancel_all_goals()
                return 'standby'
            if self.ESTOP == True:
                rospy.logdebug("ESTOP switch thrown. Cancelling all goals.")
                self.client.cancel_all_goals()
                self.waypoints.reset()
                return 'ESTOP'

            # check current state of goal (rosmsg show move_base_msgs/MoveBaseActionResult for ENUM definitions)
            current_state = self.client.get_state()
            rospy.logdebug("Current State: {}".format(current_state))
            if current_state == 0: # PENDING
                pass
            elif current_state == 1: # ACTIVE

                # label current waypoint as in progress
                if self.waypoints.current_WP.status != WaypointStatus.IN_PROGRESS:
                    self.waypoints.current_WP.status = WaypointStatus.IN_PROGRESS
                continue

            elif current_state == 2: # PREEMPTED

                rospy.logwarn("Waypoint {}: GOAL was preempted. Transitioning to STANDBY.".format(self.waypoints.current_WP.id))
                return 'standby'

            elif current_state == 3:  # SUCCEEDED
                
                rospy.logdebug("Waypoint {}: GOAL success. Transitioning to next wp: {}".format(self.waypoints.current_WP.id, self.waypoints.current_WP.id+1))

                # set the completed goal as COMPLETE
                self.waypoints.current_WP.status = WaypointStatus.COMPLETE

                # get new goal, if it exists, else return to standby
                goal = self.waypoints.get_next_goal()
                if goal is not None:
                    self.client.send_goal(goal)
                else:
                    rospy.logdebug("No more goals to navigate to. Back to STANDBY.")
                    return 'standby'

            elif current_state == 4: # ABORTED

                rospy.logwarn("Waypoint {}: GOAL was aborted. Transitioning to STANDBY.".format(self.waypoints.current_WP.id))
                self.waypoints.current_WP.status = WaypointStatus.FAILED
                return 'standby'

            elif current_state == 5: # REJECTED

                rospy.logwarn("Waypoint {}: GOAL was rejected. Trying again...".format(self.waypoints.id))
                self.waypoints.current_WP.status = WaypointStatus.FAILED
                self.client.send_goal(goal)

            elif current_state == 6: # PREEMPTING
                # this will never actually be returned by get_state()
                pass
            elif current_state == 7: # RECALLING
                # this will never actually be returned by get_state()
                pass
            elif current_state == 8: # RECALLED
                pass
            elif current_state == 9: # LOST

                rospy.logwarn("Waypoint {}: GOAL was lost. move_base may also have never had a goal to begin with. Trying again...".format(self.waypoints.id))
                self.waypoints.current_WP.status = WaypointStatus.FAILED
                self.client.send_goal(goal)

            if self.waypoints.progress == len(self.waypoints.waypoints):

                rospy.logdebug("All WPs completed. Returning to STANDBY")
                return 'standby'

            rate.sleep()

    def rc_callback(self, msg):

        if msg.switch_e is not None:
            self.ESTOP = msg.switch_e
        if msg.switch_d is not None:
            self.AUTO = msg.switch_d

def main():

    rospy.init_node("test_wps", log_level=rospy.DEBUG)

    wl = WaypointList()
    wl.read_waypoint_params('waypoints')
    
    for i in range(0, wl.size):
        rospy.logdebug(wl.get_next_waypoint())

if __name__=="__main__":
    main()