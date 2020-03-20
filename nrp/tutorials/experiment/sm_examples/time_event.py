#!/usr/bin/env python

# We need to import smach
import smach
# This package contains some state implementations that make life easier when working with the platform
import hbp_nrp_excontrol.nrp_states as states

# State machines must be named sm or state_machine, otherwise the platform will not see them
sm = smach.StateMachine(
outcomes=['FINISHED', 'ERROR', 'CONDITION_PREEMPTED',
          'ACTION_PREEMPTED', 'ACTION_ERROR'])

with sm:
    # This state will wait until the simulation time reaches 30s
    smach.StateMachine.add('CONDITION',
                           states.WaitToClockState(30),
                           {'valid': 'CONDITION', 'invalid': 'ACTION',
                            'preempted': 'CONDITION_PREEMPTED'})

    # This state will set the color of the right screen to red
    smach.StateMachine.add('ACTION',
                           states.SetMaterialColorServiceState('right_vr_screen', 'body', 'screen_glass',
                                                               'Gazebo/Red'),
                           {'succeeded': 'FINISHED', 'aborted': 'ACTION_ERROR',
                            'preempted': 'ACTION_PREEMPTED'})

# Starting, stopping and pausing of the state machine is handled by the platform