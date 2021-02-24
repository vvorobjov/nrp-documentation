#!/usr/bin/env python

import rospy
import smach
import smach_ros
import hbp_nrp_excontrol.nrp_states as states

from std_msgs.msg import String


def condition_spike_cb(user_data, sim_time, rate):

    return rate < 10

sm = smach.Concurrence(outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED'],
                       default_outcome='ERROR', input_keys=[], output_keys=[],
                       outcome_map = {'FINISHED':{'CONDITION_30':'invalid', 'CONDITION_SPIKE':'invalid'},
                                      'CONDITION_PREEMPTED':{'CONDITION_30':'preempted'},
                                      'CONDITION_PREEMPTED':{'CONDITION_SPIKE':'preempted'}})
with sm:
    smach.Concurrence.add('CONDITION_30', states.WaitToClockState(30))
    smach.Concurrence.add('CONDITION_SPIKE', states.MonitorSpikeRateState("left_wheel_spike_rate",
                                                                          condition_spike_cb))