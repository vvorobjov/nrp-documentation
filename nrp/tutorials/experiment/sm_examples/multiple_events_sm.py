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


def condition_spike_cb(user_data, sim_time, rate):
    return rate < 10


@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['finished'])
def action_spike_cb(user_data):
    # ... do something
    return 'finished'

sm_one = smach.StateMachine(outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
                                      'ACTION_PREEMPTED', 'ACTION_ERROR'])

with sm_one:
    smach.StateMachine.add('CONDITION_30',
                           states.WaitToClockState(30),
                           {'valid': 'CONDITION_30', 'invalid': 'ACTION_30',
                            'preempted': 'CONDITION_PREEMPTED'})
    smach.StateMachine.add('ACTION_30', CBState(action_30_cb),
                           {'finished': 'FINISHED'})

sm_two = smach.StateMachine(outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
                                      'ACTION_PREEMPTED', 'ACTION_ERROR'])

with sm_two:
    smach.StateMachine.add('CONDITION_SPIKE',
                           states.MonitorSpikeRateState("left_wheel_neuron_rate_monitor",
                                                        condition_spike_cb),
                           {'valid': 'CONDITION_SPIKE', 'invalid': 'ACTION_SPIKE',
                            'preempted': 'CONDITION_PREEMPTED'})
    smach.StateMachine.add('ACTION_SPIKE', CBState(action_spike_cb),
                           {'finished': 'FINISHED'})

sm = smach.Concurrence(outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED'],
                       default_outcome='ERROR', input_keys=[], output_keys=[],
                       outcome_map={'FINISHED':{'SM_ONE':'FINISHED', 'SM_TWO':'FINISHED'},
                                    'CONDITION_PREEMPTED': {'SM_ONE':'CONDITION_PREEMPTED',
                                                            'SM_TWO':'CONDITION_PREEMPTED'}})

with sm:
    smach.Concurrence.add('SM_ONE', sm_one)
    smach.Concurrence.add('SM_TWO', sm_two)