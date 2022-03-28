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
        smach.State.__init__(self, outcomes=['boot_success', 'None'])

        # TODO: define message callbacks for topics to watch and throw flags
        # what needs to be verified before we can begin?

        self._velodyne_flag = False 
        self._odom_flag = False
        self._cam_flag = True   # TODO: we shouldn't need to hard-code a true here
        self._imu_flag= False
        self._gps_flag = False

        # Odom subscriber
        rospy.Subscriber('/bowser2/odom', nav.Odometry, callback=self.odom_callback)
        # Lidar subscriber
        rospy.Subscriber('/velodyne', sens.PointCloud2, callback=self.velodyne_callback)
        # Depth camera
        rospy.Subscriber('/bowser2/bowser2_dc/depth/camera_info', sens.CameraInfo, callback=self.cam_callback)
        # Imu Subscriber
        rospy.Subscriber('/bowser2/imu', sens.Imu, callback=self.imu_callback)
        # GPS Subscriber
        rospy.Subscriber('/fix', sens.NavSatFix, callback=self.gps_callback)

        # received ACK from all software modules (define list in XML/YAML format?)

    def execute(self, userdata):

        # configure timer to output status of subscribers every 2 secs
        status_timer = rospy.Timer(rospy.Duration(2), self.timer_status_callback)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self._velodyne_flag and self._odom_flag and self._cam_flag and self._imu_flag and self._gps_flag:
                rospy.logdebug("All sources up. Transitioning to STANDBY.")
                # end the status timer
                status_timer.shutdown()
                return 'boot_success'

            rate.sleep()

    def timer_status_callback(self, event):

        rospy.logdebug(
            "\nODOM: \t\t{}\n".format(self._odom_flag) + \
            "LIDAR: \t\t{}\n".format(self._velodyne_flag) + \
            "CAMERA: \t\t{}\n".format(self._cam_flag) + \
            "IMU: \t\t{}\n".format(self._imu_flag) + \
            "GPS: \t\t{}\n".format(self._gps_flag)
        )

    def cam_callback(self, data):
        
        self._cam_flag = True

    def velodyne_callback(self, data):

        self._velodyne_flag = True

    def odom_callback(self, data):

        # rospy.logdebug("ODOM CALLBACK")

        self._odom_flag = True

    def imu_callback(self, data):

        self._imu_flag = True

    def gps_callback(self, data):

        self._gps_flag = True

class Standby(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['got_pose', 'rc_preempt'],
                                   output_keys=['gps_x', 'gps_y', 'gps_z', 'gps_x0', 'gps_y0', 'gps_z0', 'gps_w0'])

        self.got_pose = False
        self.data_copy = None

        # cmd_sub = rospy.Subscriber('/cmd_pose', geom.PoseStamped, callback = self.cmd_callback)

    def execute(self, userdata):

        # wait until execute to initialize subscribers so that multiple states can listen to same topic names without clashing
        cmd_sub = rospy.Subscriber('/cmd_pose', geom.PoseStamped, callback = self.cmd_callback)

        rate = rospy.Rate(20)

        while not rospy.is_shutdown():

            if self.got_pose:
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

            rate.sleep()

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
                                   input_keys=['way_x', 'way_y', 'way_z', 'way_x0', 'way_y0', 'way_z0', 'way_w0'],
                                   output_keys=[])
        
        self.waypoint = rospy.Publisher('/move_base_simple/goal', geom.PoseStamped, queue_size=10)
        self.geom_temp = geom.PoseStamped()

    def execute(self, userdata):
        self.geom_temp.pose.position.x = userdata.way_x
        self.geom_temp.pose.position.y = userdata.way_y
        self.geom_temp.pose.position.z = userdata.way_z

        self.geom_temp.pose.orientation.x = userdata.way_x0
        self.geom_temp.pose.orientation.y = userdata.way_y0
        self.geom_temp.pose.orientation.z = userdata.way_z0
        self.geom_temp.pose.orientation.w = userdata.way_w0

        rospy.logdebug(self.geom_temp)

        while not rospy.is_shutdown():
            self.waypoint.publish(self.geom_temp)

        return 'End'

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
        smach.State.__init__(self, outcomes=[],
                                   input_keys=['reason'])

    def execute(self, userdata):

        # kill the ROS node
        # http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
        rospy.signal_shutdown(userdata.reason)

        # if userdata.end_status == 'success':
        #     return 'end_success'
        # elif userdata.end_status == 'err':
        #     return 'end_err'

def main():

    # initialize ROS node
    rospy.init_node('rover_sm', anonymous=True, log_level=rospy.DEBUG)

    # create state machine with outcomes
    sm = smach.StateMachine(outcomes = ['success', 'err'])

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
            transitions={'got_pose':'WAYPOINT', 'rc_preempt':'MANUAL'},
            remapping={'gps_x':'x_pos',
                        'gps_y':'y_pos',
                        'gps_z':'z_pos',
                        'gps_xO':'x_ori',
                        'gps_yO':'y_ori',
                        'gps_zO':'z_ori',
                        'gps_wO':'w_ori'})
        smach.StateMachine.add('WAYPOINT',
            Waypoint(),
            transitions={'End': 'END'},
            remapping={'way_x':'x_pos',
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
        smach.StateMachine.add('END',
            End(),
            transitions={},
            remapping={'reason': 'end_reason'})


    # create an introspection server for debugging transitions

    outcome = sm.execute()

    rospy.spin()

if __name__ == '__main__':
    main()