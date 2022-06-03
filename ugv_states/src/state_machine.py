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

# import states
from Auto import Auto
from Boot import Boot
from Standby import Standby
from Estop import Estop
from End import End
from Warn import Warn
from Manual import Manual

def main():

    # initialize ROS node
    rospy.init_node('rover_sm', anonymous=True, log_level=rospy.DEBUG)

    # create state machine with outcomes
    sm = smach.StateMachine(outcomes = ['success', 'err', 'end'])

    # declare userdata
    sm.userdata.end_reason = 'success'
    sm.userdata.auto_success = None

    # define states within sm
    with sm:
        smach.StateMachine.add('BOOT',
            Boot(),
            transitions={'boot_success':'STANDBY', 'None':'END'},
            remapping={})
        smach.StateMachine.add('STANDBY',
            Standby(),
            transitions={'rc_preempt':'MANUAL', 'ESTOP':'ESTOP', 'AUTO':'AUTO'},
            remapping={})
        smach.StateMachine.add('MANUAL',
            Manual(),
            transitions={'rc_un_preempt':'STANDBY', 'error':'END'},
            remapping={})
        # smach.StateMachine.add('WARN',
        #     Warn(),
        #     transitions={'reset':'BOOT', 'standby':'STANDBY', 'end':'END'},
        #     remapping={})
        smach.StateMachine.add('ESTOP',
            Estop(),
            transitions={'standby':'STANDBY', 'end':'END'},
            remapping={})
        smach.StateMachine.add('AUTO',
            Auto(),
            transitions={'standby':'STANDBY', 'ESTOP':'ESTOP', 'error':'END'},
            remapping={'auto_success':'auto_success'})
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