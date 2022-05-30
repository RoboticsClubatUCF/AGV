#!/usr/bin/env python

# ROS system imports
import rospy
import actionlib
import smach, smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal

# ROS messages
import std_msgs.msg as std
import nav_msgs.msg as nav
import sensor_msgs.msg as sens
import geometry_msgs.msg as geom
import move_base_msgs.msg as move_base
import ugv_msg.msg as ugv

from Auto import Auto

class DataStream():

    def __init__(self, label, msg_type, topic, hz=None):

        self.label = label
        self.msg_type = msg_type
        self.topic = topic
        self.hz = hz if hz else None

        self.flag = False        

class Boot(smach.State):
    """
    BOOT ensures that we have all necessary data streams (from sensors/rc control/whatever) before passing us to STANDBY.
    Any other 'preflight' checks should be done here, as well.
    """

    def __init__(self):
        smach.State.__init__(self, outcomes=['boot_success', 'None'])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        # define all required data streams
        self.streams = [DataStream('RC', ugv.RC, '/choo_2/rc'),
                        DataStream('ODOM', nav.Odometry, '/choo_2/odom'),
                        DataStream('LIDAR', sens.PointCloud2, '/velodyne_points'),
                        # DataStream('IMU', sens.Imu, '/vectornav/IMU'),
                        DataStream('GPS', sens.NavSatFix, '/choo_2/fix')]

        # create subscriber for each data stream
        for stream in self.streams:
            rospy.Subscriber(stream.topic, stream.msg_type, callback=self.stream_callback, callback_args=(stream, stream.topic))

    def execute(self, userdata):

        self.state_pub.publish("BOOT")

        # configure timer to output status of subscribers every 2 secs
        status_timer = rospy.Timer(rospy.Duration(2), self.timer_status_callback)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            # transition to STANDBY if all sensor streams are up
            if all(stream.flag == True for stream in self.streams):
                rospy.logdebug("All sources up. Transitioning to STANDBY.")
                # end the status timer
                status_timer.shutdown()
                return 'boot_success'

            rate.sleep()

    def timer_status_callback(self, event):

        rospy.logdebug("Sensor stream status:")
        for stream in self.streams:
            rospy.logdebug("{}\t({}):\t\t{} ".format(stream.label, stream.topic, stream.flag))
        rospy.logdebug("---------------------------------------")

    def stream_callback(self, msg, args):

        # unpack args tuple
        stream = args[0]
        topic = args[1]

        if msg._type == stream.msg_type._type and topic == stream.topic:
            stream.flag = True

class Standby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['got_pose', 'rc_preempt', 'ESTOP', 'AUTO'],
                                   output_keys=['frame_id', 'gps_x', 'gps_y', 'gps_z', 'gps_x0', 'gps_y0', 'gps_z0', 'gps_w0'])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        self.got_pose = False
        self.data_copy = None

        self.AUTO = False
        self.ESTOP = False

    def execute(self, userdata):

        self.state_pub.publish("STANDBY")

        # wait until execute to initialize subscribers so that multiple states can listen to same topic names without clashing
        rc_sub = rospy.Subscriber("/choo_2/rc", ugv.RC, callback=self.rc_callback)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            # transition to pose, fill userdata with pose
            if self.got_pose:
                # fill header fields
                userdata.frame_id = self.data_copy.header.frame_id
                # fill pose fields
                userdata.gps_x = self.data_copy.pose.position.x
                userdata.gps_y = self.data_copy.pose.position.y
                userdata.gps_z = self.data_copy.pose.position.z
                # fill orientation fields
                userdata.gps_x0 = self.data_copy.pose.orientation.x
                userdata.gps_y0 = self.data_copy.pose.orientation.y
                userdata.gps_z0 = self.data_copy.pose.orientation.z
                userdata.gps_w0 = self.data_copy.pose.orientation.w
                return 'got_pose'

            if self.ESTOP:
                self.ESTOP = False
                return 'ESTOP'
            elif self.AUTO:
                self.AUTO = False
                return 'AUTO'

            rate.sleep()

    def rc_callback(self, msg):

        if msg.switch_e:
            self.ESTOP = msg.switch_e
        if msg.switch_d:
            self.AUTO = msg.switch_d

    # Called when movement data is received
    def cmd_callback(self, data):
        self.data_copy = data
        self.got_pose = True
        return None


class Waypoint(smach.State):

    ''' pose_target contains the pose target that initiated the switch to waypoint 
        NOT guaranteed to remain the pose_target during operation '''

    def __init__(self):
        smach.State.__init__(self, outcomes=['End'],
                                   input_keys=['frame_id', 'way_x', 'way_y', 'way_z', 'way_x0', 'way_y0', 'way_z0', 'way_w0'],
                                   output_keys=[])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

    def execute(self, userdata):

        self.state_pub.publish("WAYPOINT")

        # wait until execute to initialize subscribers so that multiple states can listen to same topic names without clashing
        cmd_sub = rospy.Subscriber('/cmd_pose', geom.PoseStamped, callback = self.pose_callback)

        # create a client to the movebase action server
        self.client = actionlib.SimpleActionClient("move_base", move_base.MoveBaseAction)
        # make sure we have connection to client server before continuing, timeout after 10s
        self.client.wait_for_server(timeout=rospy.Duration(10))

        # create MoveBaseGoal message from userdata
        self.goal_temp = move_base.MoveBaseGoal()
        # fill header fields
        self.goal_temp.target_pose.header.frame_id = userdata.frame_id
        self.goal_temp.target_pose.header.stamp = rospy.Time.now()
        # fill pose fields
        self.goal_temp.target_pose.pose.position.x = userdata.way_x
        self.goal_temp.target_pose.pose.position.y = userdata.way_y
        self.goal_temp.target_pose.pose.position.z = userdata.way_z
        # fill orientation fields
        self.goal_temp.target_pose.pose.orientation.x = userdata.way_x0
        self.goal_temp.target_pose.pose.orientation.y = userdata.way_y0
        self.goal_temp.target_pose.pose.orientation.z = userdata.way_z0
        self.goal_temp.target_pose.pose.orientation.w = userdata.way_w0

        # send goal
        self.client.send_goal(self.goal_temp)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():

            current_state = self.client.get_state()
            if current_state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal Succeeded.\n {}".format(self.client.get_goal_status_text()))
                return 'nav_finish'
            elif current_state == actionlib.GoalStatus.REJECTED or \
                 current_state == actionlib.GoalStatus.ABORTED:
                rospy.logerr("Goal returned an error:\n\t {}".format(self.client.get_goal_status_text))
                return 'error'
            elif current_state == actionlib.GoalStatus.LOST:
                rospy.logerr("Goal state returned LOST. Are we sure we sent a goal?")

            # self.waypoint.publish(self.geom_temp)
            # pass

            # return 'End'

    def pose_callback(self, data):
        pass

class Manual(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['rc_un_preempt', 'resume_waypoint', 'error'])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

        # subscribe to RC commands
        # TODO: make/find an RC channels message
        rospy.Subscriber('/choo_2/rc', std.String, callback=self.rc_callback)

        # publish motor commands for the base_controller to actuate
        self.motor_pub = rospy.Publisher('/cmd_vel', geom.Twist, queue_size=10)
    
    def execute(self, userdata):

        self.state_pub.publish("MANUAL")

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

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

    def execute(self, userdata):

        self.state_pub.publish("WARN")

        pass

class Estop(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['reset', 'end'],
                                   output_keys=[])

        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

    def execute(self, userdata):

        self.state_pub.publish("ESTOP")

        pass

class End(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['success', 'err', 'end'],
                                   input_keys=['reason'])
        
        self.state_pub = rospy.Publisher('/choo_2/state', std.String, queue_size=1)

    def execute(self, userdata):

        self.state_pub.publish("END")
        # kill the ROS node
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown

        return 'end'

        # rospy.signal_shutdown(userdata.reason)

        # if userdata.end_status == 'success':
        #     return 'end_success'
        # elif userdata.end_status == 'err':
        #     return 'end_err'

def main():

    # initialize ROS node
    rospy.init_node('rover_sm', anonymous=True, log_level=rospy.DEBUG)

    # create state machine with outcomes
    sm = smach.StateMachine(outcomes = ['success', 'err', 'end'])

    # declare userdata
    sm.userdata.x_pos = 0
    sm.userdata.y_pos = 0
    sm.userdata.z_pos = 0

    sm.userdata.end_reason = 'success'

    sm.userdata.x_ori = 0
    sm.userdata.y_ori = 0
    sm.userdata.z_ori = 0
    sm.userdata.w_ori = 0

    # define states within sm
    with sm:
        smach.StateMachine.add('BOOT',
            Boot(),
            transitions={'boot_success':'STANDBY', 'None':'END'},
            remapping={})
        smach.StateMachine.add('STANDBY',
            Standby(),
            transitions={'got_pose':'WAYPOINT', 'rc_preempt':'MANUAL', 'ESTOP':'END', 'AUTO':'AUTO'},
            remapping={ 'frame_id':'frame_id',
                        'gps_x':'x_pos',
                        'gps_y':'y_pos',
                        'gps_z':'z_pos',
                        'gps_xO':'x_ori',
                        'gps_yO':'y_ori',
                        'gps_zO':'z_ori',
                        'gps_wO':'w_ori'})
        smach.StateMachine.add('WAYPOINT',
            Waypoint(),
            transitions={'End': 'END'},
            remapping={ 'frame_id':'frame_id',
                        'way_x':'x_pos',
                        'way_y':'y_pos',
                        'way_z':'z_pos',
                        'way_x0':'x_ori',
                        'way_y0':'y_ori',
                        'way_z0':'z_ori',
                        'way_w0':'w_ori'})
        smach.StateMachine.add('MANUAL',
            Manual(),
            transitions={'rc_un_preempt':'STANDBY', 'resume_waypoint':'WAYPOINT', 'error':'END'},
            remapping={})
        # smach.StateMachine.add('WARN',
        #     Warn(),
        #     transitions={'reset':'BOOT', 'standby':'STANDBY', 'end':'END'},
        #     remapping={})
        smach.StateMachine.add('ESTOP',
            Estop(),
            transitions={'reset':'STANDBY', 'end':'END'},
            remapping={})
        smach.StateMachine.add('AUTO',
            Auto(),
            transitions={'standby':"STANDBY", 'ESTOP':"ESTOP", 'error':"END"},
            remapping={})
        smach.StateMachine.add('END',
            End(),
            transitions={},
            remapping={'reason': 'end_reason'})


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('ugv_states', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()
    sis.stop()

if __name__ == '__main__':
    main()