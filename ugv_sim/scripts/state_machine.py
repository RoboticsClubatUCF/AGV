#!/usr/bin/env python3

# ROS system imports
import rospy
import actionlib
import smach, smach_ros

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sens
import geometry_msgs.msg as geom
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class Boot(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error', 'boot_success'])

        # TODO: define message callbacks for topics to watch and throw flags
        # what needs to be verified before we can begin?

        # RX HEARTBEAT
        rospy.Subscriber('/heartbeat', rov.Heartbeat, callback=self.hb_callback)
        self._hb_flag = False

        # RX data from all sensor stream topics

        # received ACK from all software modules (define list in XML/YAML format?)

    def execute(self, userdata):

        while not rospy.is_shutdown():

            if self._hb_flag:
                return 'boot_success'

            # what constitutes an error?

    def hb_callback(self, data):

        self._hb_flag = True

class Standby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['got_pose', 'rc_preempt', 'error', 'end'],
                                   output_keys=['pose_target', 'end_status', 'end_reason'] )

        # flags
        self._rc_preempt = False
        self._pose_preempt = False
        self._end = False

        self._pose_target = geom.PoseStamped()

        cmd_sub = rospy.Subscriber('/command', rov.Cmd, callback=self.cmd_callback)

    def execute(self, userdata):

        while not rospy.is_shutdown():

            # if we received motor commands, 
            if self.rc_preempt:
                rospy.logdebug("Standby preempted by RC command.")
                return 'rc_preempt'
            # if we received a pose in Command msg, pass that as output of state
            if self.pose_preempt:
                userdata.pose_target = self._pose_target
                rospy.logdebug("Standby preempted by pose target.")
                return 'got_pose'

            # check for errors
            # if err:
            #   userdata.end_reason = 'Fatal Error'
            #   userdata.end_status = 'err'
            #   return 'end'

            # check for user-initiated end
            if self._end:
                rospy.logdebug('Standby preempted by end signal.')
                userdata.end_reason = 'User initiated end state.'
                userdata.end_status = 'success'
                return 'end'


    def cmd_callback(self, data):

        ''' TODO: check for override flag in RC message, throw rc_preempt '''

        # assumes that pose_target field of Command msg is empty if we don't want waypoint navigation
        if data.pose_target is not None:
            self._pose_target = data.pose_target
            self._pose_preempt = True

        if data.rc_preempt is not None:
            if data.rc_preempt.data:
                self._rc_preempt = True
            
        # check for the shutdown flag    
        if data.shutdown is not None:
            if data.shutdown.data:
                self._end = True


class Waypoint(smach.State):

    ''' pose_target contains the pose target that initiated the switch to waypoint 
        NOT guaranteed to remain the pose_target during operation '''

    def __init__(self):
        smach.State.__init__(self, outcomes=['nav_finish', 'rc_preempt', 'error'],
                                   input_keys=['pose_target'],
                                   output_keys=['status'])

        # create the client that will connect to move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
        # listen for pose updates that will change our pose_target
        pose_sub = rospy.Subscriber('/pose_updates', geom.PoseStamped, callback=self.pose_callback)
        self.pose_update = False

        # the pose target that WAYPOINT will use for navigation
        self._pose_target = None

    def execute(self, userdata):

        ''' manages the lifecycle of calls to the autonomy stack '''

        # get input pose target (pose that caused us to transition to Waypoint)
        self._pose_target = userdata.pose_target
        self._pose_update = True

        # make sure we have connection to client server before continuing
        self.client.wait_for_server()

        while not rospy.is_shutdown():

            # will need to use an actionlib connection to move_base, like below:
            # https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/
            # this approach may require multithreading to not block the main loop

            # for a non-blocking method: https://answers.ros.org/question/347823/action-client-wait_for_result-in-python/
            # 1. send goal with send_goal()
            # 2. query action server for state with getState()
            # 3. if we're in the appropriate state, getResult()
            # api docs: http://docs.ros.org/melodic/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html

            # if we received a pose update, create a new goal
            if self._pose_update:
                
                # goal = MoveBaseGoal()

                # goal.target_pose.header.frame_id = "map"
                # goal.target_pose.header.stamp = rospy.Time.now()
                # goal.target_pose.pose.position.x = 0.5
                # goal.target_pose.pose.orientation.w = 1.0

                # client.send_goal(goal)
                # wait = client.wait_for_result()
                # if not wait:
                #     rospy.logerr("Action server not available!")
                #     rospy.signal_shutdown("Action server not available!")
                # else:
                #     return client.get_result()

                # reset the flag
                self._pose_update = False

    def pose_callback(self, data):
        # update internal pose target
        self._pose_target = data.pose
        # set flag to true
        self._pose_update = True
    
class Manual(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['rc_un_preempt', 'resume_waypoint', 'error'])

        # subscribe to RC commands
        # TODO: make/find an RC channels message
        rospy.Subscriber('/rc', std.String, callback=self.rc_callback)

        # publish motor commands for the base_controller to actuate
        self.motor_pub = rospy.Publisher('/cmd_vel', geom.Twist, queue_size=10)
    
    def execute(self, userdata):

        while not rospy.is_shutdown():
            pass

    def rc_callback(self, data):

        # convert raw RC to twist (if that's the approach we're taking)

        # publish on cmd_vel with self.motor_pub
        pass

class Warn(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['reset', 'standby', 'end'],
                                   output_keys=['end_status', 'end_reason'])

    def execute(self, userdata):

        pass

class End(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['end_success', 'end_err'],
                                   input_keys=['end_status', 'end_reason'])

    def execute(self, userdata):

        # kill the ROS node
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
        rospy.signal_shutdown(userdata.end_reason)

        if userdata.end_status == 'success':
            return 'end_success'
        elif userdata.end_status == 'err':
            return 'end_err'

def main():

    # initialize ROS node
    rospy.init_node('rover_sm', anonymous=True)

    # TODO: publisher for Telemetry message

    # create state machine with outcomes
    sm = smach.StateMachine(outcomes=['success', 'err'])

    # declare userdata
    sm.userdata.pose_target = geom.PoseStamped()

    # define states within sm
    with sm:
        smach.StateMachine.add('BOOT', 
            Boot(), 
            transitions={'error':'WARN', 'boot_success':'STANDBY'}, 
            remapping={})
        smach.StateMachine.add('STANDBY',
            Standby(),
            transitions={'got_pose':'WAYPOINT', 'rc_preempt':'MANUAL', 'error':'WARN', 'end':'END'},
            remapping={'pose_target':'pose_target', 'end_status':'end_status', 'end_reason':'end_reason'})
        smach.StateMachine.add('WAYPOINT',
            Waypoint(),
            transitions={'nav_finish':'STANDBY', 'rc_preempt':'MANUAL', 'error':'WARN'},
            remapping={'pose_target':'pose_target', 'status':'waypoint_status'})
        smach.StateMachine.add('MANUAL',
            Manual(),
            transitions={'rc_un_preempt':'STANDBY', 'resume_waypoint':'WAYPOINT', 'error':'WARN'},
            remapping={})
        smach.StateMachine.add('WARN',
            Warn(),
            transitions={'reset':'BOOT', 'standby':'STANDBY', 'end':'END'},
            remapping={})
        smach.StateMachine.add('END',
            End(),
            transitions={'end_success':'success', 'end_err':'err'},
            remapping={'end_status':'end_status', 'end_reason':'end_reason'})


    # create an introspection server for debugging transitions
    introspect = smach_ros.IntrospectionServer('rover_sm_info', sm, '/SM_ROOT')
    introspect.start()

    outcome = sm.execute()

    rospy.spin()
    introspect.stop()

if __name__ == '__main__':
    main()
