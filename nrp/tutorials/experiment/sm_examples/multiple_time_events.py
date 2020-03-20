#!/usr/bin/env python

import rospy
import smach
from smach import CBState
import smach_ros
import hbp_nrp_excontrol.nrp_states as states

from std_msgs.msg import String


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_30_cb(user_data):
    # ... do something
    return 'finished'


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_60_cb(user_data):
    # ... do something
    return 'finished'

sm = smach.StateMachine(
outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
          'ACTION_PREEMPTED', 'ACTION_ERROR'])

with sm:
    smach.StateMachine.add('CONDITION_30',
                           states.WaitToClockState(30),
                           {'valid': 'CONDITION_30', 'invalid': 'ACTION_30',
                            'preempted': 'CONDITION_PREEMPTED'})
    smach.StateMachine.add('ACTION_30', CBState(action_30_cb),
                           {'finished': 'CONDITION_60'})
    smach.StateMachine.add('CONDITION_60',
                           states.WaitToClockState(60),
                           {'valid': 'CONDITION_60', 'invalid': 'ACTION_60',
                            'preempted': 'CONDITION_PREEMPTED'})
    smach.StateMachine.add('ACTION_60', CBState(action_60_cb),
                           {'finished': 'FINISHED'})