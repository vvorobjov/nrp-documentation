Tutorial: Interacting with an Experiment from the Virtual Coach
===============================================================

In the previous section we successfully launched a simulation of the Template Husky experiment. In this tutorial, we will interact with the simulation through the Virtual Coach.

Getting and Setting Simulation States
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
When an experiment is launched, it is initially in the `paused` state. This means the experiment has to be explicitly started. In general, we can query the simulation state by calling:

.. code-block:: python

    sim.get_state()

The state of the simulation is returned as a string. In this case calling get_state() will return `paused`. The possible states we can set a simulation to are `paused`, `started` and `stopped` using these calls respectively:

.. code-block:: python

    sim.pause()
    sim.start()
    sim.stop()


Transfer Functions
^^^^^^^^^^^^^^^^^^
From the Virtual Coach, we can view transfer functions, modify them, delete them, or add new ones. The transfer functions unique identifiers are their function names. To know the names of the transfer functions defined in an experiments, we can call:

.. code-block:: python

    sim.print_transfer_functions()

which would just print out the names of the transfer functions. In the case of the Template Husky experiment, this call will print out: `turn_around`, `csv_spike_monitor`, `all_neurons_spike_monitor`, `grab_image`, and `csv_joint_state_monitor`. To actually see the code of a transfer function, we can use the get_transfer_function call, which takes the transfer function name as a parameter and returns the code in a string. We will get the code of the turn-around transfer function and store it in a variable:

.. code-block:: python

    turn_around_tf = sim.get_transfer_function('turn_around')

If you're using a jupyter notebook, you can use the `%load` command to load the transfer function code in a cell. This will allow you to see the code syntax highlighted and properly indented and will make it easier to modify the functions:

.. code-block:: python

    %load turn_around_tf

This should load this function in a new cell in the jupyter notebook:

.. code-block:: python

    import hbp_nrp_cle.tf_framework as nrp
    from hbp_nrp_cle.robotsim.RobotInterface import topic
    import geometry_msgs.msg
    @nrp.MapSpikeSink("output_neuron", nrp.brain.neurons[1], nrp.leaky_integrator_alpha)
    @nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
    # Example TF: get output neuron voltage and output constant on robot actuator. You could do something with the voltage here and command the robot accordingly.
    def turn_around(t, output_neuron):
        voltage=output_neuron.voltage
        return geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(0,0,0),
                                       angular=geometry_msgs.msg.Vector3(0,0,5))

You can modify the code however you want and then save it as a string. Here we'll just increase the angular velocity by modifying the last line in the above code and replacing the number 5 with 10. We will save the modified code as a string in the variable turn_around_tf:

.. code-block:: python

    turn_around_tf = """
    import hbp_nrp_cle.tf_framework as nrp
    from hbp_nrp_cle.robotsim.RobotInterface import topic
    import geometry_msgs.msg
    @nrp.MapSpikeSink("output_neuron", nrp.brain.neurons[1], nrp.leaky_integrator_alpha)
    @nrp.Neuron2Robot(Topic('/husky/cmd_vel', geometry_msgs.msg.Twist))
    # Example TF: get output neuron voltage and output constant on robot actuator. You could do something with the voltage here and command the robot accordingly.
    def turn_around(t, output_neuron):
        voltage=output_neuron.voltage
        return geometry_msgs.msg.Twist(linear=geometry_msgs.msg.Vector3(0,0,0),
                                       angular=geometry_msgs.msg.Vector3(0,0,10))
    """

This modified transfer function will only make the robot spin faster in this experiment. If you open your frontend web cockpit and join the running experiment, you will see the robot spinning faster once we actually apply the transfer function. To apply the transfer function we use the call edit_transfer_function which takes as parameters the name of the transfer function to be modified and the modified code.

.. code-block:: python

    sim.edit_transfer_function('turn_around', turn_around_tf)

The Virtual Coach will maintain the simulation state after setting the transfer function. This means that if the simulation was running, the Virtual Coach will modify the transfer function and then automatically start the simulation again.

As a user you can also delete transfer functions from the Virtual Coach. You just need to provide the name of the transfer function and use it in the following call:

.. code-block:: python

    sim.delete_transfer_function('turn_around')

This will delete the turn_around transfer function we just modified. After that you will notice that the robot stopped spinning since the transfer function responsible for that behavior has been deleted. If you want more proof that the transfer function has been deleted, you can revisit the print_transfer_functions call and make sure that it doesn't print out turn_around.

We can also add new transfer functions. For this we only need to provide the transfer function code as a string parameter to the add_transfer_function function. We don't have to provide a name since the name will just be the function's name. Remember that transfer functions definition names have to be unique, so duplicate function names will result in errors. Here we'll create a new transfer function with the default transfer function code you get when adding a new transfer function from the web cockpit.

.. code-block:: python

    new_transfer_function = """
    @nrp.Robot2Neuron()
    def transferfunction_0(t):
        if (t%2 < 0.02):
            clientLogger.info('Time: ', t)
    """

    sim.add_transfer_function(new_transfer_function)

This transfer function will log the simulation time to the log console every two seconds.

State Machines
^^^^^^^^^^^^^^
Through the Virtual Coach, users can interact with the simulation state machines the same way they can with the transfer functions. Currently we have only one experiment that contains a state machine. Let's start it and see how we can interact with the state machines.

.. code-block:: python

    sim = vc.launch_experiment('ScreenSwitchingHuskyExperimnent')

After the experiment has been started, we can retrieve the names of the defined state machines.

.. code-block:: python

    sim.print_state_machines()

This call should print out `HuskyAwareScreenControlling`. To retrieve the code of the state machine, we will have to use its name we just got.

.. code-block:: python

    sm = sim.get_state_machine('HuskyAwareScreenControlling')

Since state machines are also python scripts, we can load them in jupyter notebooks with the `%load` command like we did with the transfer functions. Additionally, we can also edit and delete them, or add new ones, exactly like we interact with transfer functions. Below are the calls for editing, deleting and adding state machines.

.. code-block:: python

    sim.edit_state_machine(state_machine_name, state_machine_code)
    sim.delete_state_machine(state_machine_name)
    sim.add_state_machine(state_machine_name, state_machine_code)

The only difference between interacting with state machines and transfer functions is that the state machines' are not the python function names. Therefore, when adding a new state machine, the user has to explicitly give it a name.


Getting CSV Data
^^^^^^^^^^^^^^^^
Currently in all our template experiments, Spike and Joint data are being saved to csv files. Data being saved to csv files are, however, not limited to Spike and Joint data. You can access all csv data from the Virtual Coach and plot or analyze the data. To know what kind of data is being saved to csv files in an experiment, you can print out the names of the csv files first using this call:

.. code-block:: python

    sim.print_csv_file_names()

In the case of the Template Husky experiment, this will print out `all_spikes.csv` and `all_joints_positions.csv`. We can now get the data from any one of these files. Note that these files will be populated only if a simulation has been running. Here we will get and print out the Spike data:

.. code-block:: python

    spikes = sim.get_csv_data('all_spikes.csv')
    pprint(spikes)
    [[u'id', u'time', u'Simulation_reset'],
     [u'8.0', u'0.10000000000000001', u'RESET'],
     [u'10.0', u'2.6000000000000001', u''],
     [u'8.0', u'57.200000000000003', u'']]

In the code snippet above you can notice the additional `Simulation_reset` column in that automatically keeps track of 'reset' events.

We can also write our own custom functions to plot the data we got. The following is a custom function that will plot each spike from the csv file as a blue dot. Note also that the first line in the csv data is a header that need to be accounted for when plotting.

.. code-block:: python

    import matplotlib.pyplt as plt
    plt.gca()
    plt.ylim(2, 12)
    plt.yticks(range(3, 11))
    plt.xlabel('Time [ms]')
    plt.ylabel('Neuron ID')
    plt.plot([float(spike[1]) for spike in spikes[1:], [int(float(spike[0])) for spike in spikes[1:]], 'bo')

Reset Functionality
^^^^^^^^^^^^^^^^^^^
It is also possible to reset certain aspects of the simulation from the Virtual Coach, exactly as it is possible from the web cockpit. There are four reset types possible from the Virtual Coach: `Robot Frame`, `Environment`, `Brain`, and the `Full Simulation`. You can reset all simulation aspects with the same call:

.. code-block:: python

    sim.reset('robot_pose')
    sim.reset('world')
    sim.reset('brain')
    sim.reset('full')


If you want to look at more concrete example experiments run from the Virtual Coach, you can check out the hbp_nrp_virtual_coach/examples directory.
