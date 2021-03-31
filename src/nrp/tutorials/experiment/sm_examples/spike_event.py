def condition_spike_cb(user_data, sim_time, rate):
    return rate < 10

smach.StateMachine.add('CONDITION',
                       states.MonitorSpikeRateState("left_wheel_neuron_rate_monitor",
                                                    condition_spike_cb),
                       {'valid': 'CONDITION', 'invalid': 'ACTION',
                        'preempted': 'CONDITION_PREEMPTED'})
