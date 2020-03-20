Virtual Coach API
=================

This package contains the Virtual Coach implementation and interfaces to the Neurorobotics Platform backend services.


VirtualCoach Module
-------------------

Virtual Coach main entry point for interacting with the experiment list and launching simulations.

class *hbp_nrp_virtual_coach*.virtual_coach.\ **VirtualCoach**\ (\ *environment=None*, *oidc_username=None*, *storage_username=None*\ )

    Bases: object

    Provides the user with methods for interacting with the experiment list, providing functionality similar to the graphical frontend experiment list webpage. Allows the user to view available experiments, query currently running experiments, launch a simulation, and more.


**clone_experiment_to_storage**\ (\ *exp_id*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Attempts to clone an experiment to the Storage Server. Only works if the Virtual Coach was instantiated with Storage Server support, i.e. Storage Server credentials.

    **Parameters**

        + **exp_id** The id of the experiment to be cloned.

**delete_cloned_experiment**\ (\ *exp_id*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Attempts to delete a cloned experiment from the Storage Server

    **Parameters**

        + **exp_id** The id of the experiment to be deleted.

**import_experiment**\ (\ *path*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Imports an experiment folder, possibly a zipped folder, into user Storage

    **Parameters**

        + **path** String that contains the path to the folder or to the zip file to be imported (required).


**launch_experiment**\ (\ *experiment_id*, *server=None*, *reservation=None*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Attempts to launch a simulation with the given parameters. If no server is explicitly given then all available backend servers will be tried. Only cloned experiments to the Storage Server can be launched.

    **Parameters:**

        + **experiment_id** The short name of the experiment configuration to launch (e.g. ExDTemplateHusky).
        + **server** (optional) The full name of the server backend to launch on, if none is provided, then all backend servers will be checked.
        + **reservation** (optional) A cluster reservation string if the user has reserved specific resources, otherwise use any available resources.


**print_available_servers**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Prints a list of the available backend servers that are currently not running a simulation.


**print_templates**\ (\ *dev=False*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Prints a table of the list of experiment templates available on the backend environment. The printed list is sorted by the experiment title in the same way as the frontend webpage.

    **Parameters:**

         + **dev** (optional) A boolean flag if all development maturity experiments should be printed in addition to the production experiments.


**print_cloned_experiments**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Prints the list of the cloned experiments' names. Only works if the Virtual Coach was instantiated with Storage Server Support, i.e. Storage Server credentials.


**print_running_experiments**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Prints a table of currently running experiments and relevant details (if any).


Simulation Module
-----------------

An interface to launch or control a simulation instance.

class *hbp_nrp_virtual_coach*.simulation.\ **Simulation**\ (\ *oidc_client*, *config*\ )

    Bases: **object**

    Provides an interface to launch or control a simulation instance.


**add_state_machine**\ (\ *state_machine_name*, *state_machine*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Adds a new State Machine to the simulation.

    **Parameters:**

        + **state_machine_name:** A string containing the name of the state machine to be edited.
        + **state_machine:** A string containing the new state machine code.


**add_transfer_function**\ (\ *transfer_function*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Adds a new transfer function to the simulation.

    **Parameters**

        + **transfer_function** A string containing the new transfer function code.


**delete_state_machine**\ (\ *state_machine_name*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Deletes a state machine.

    **Parameters:**

        + **state_machine_name** A string containing the name of the state machine to be deleted.


**delete_transfer_function**\ (\ *transfer_function_name*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Deletes a transfer function.

    **Parameters**

        + **transfer_function_name** A string containing the name of the transfer function to be deleted.


**edit_brain**\ (\ *brain_script*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Edits the brain script defined in the simulation and keeps the defined populations. Calling this method will not change brain populations.

    **Parameters:**

         + **brain_script** A string containing the new pyNN script.


**edit_populations**\ (\ *populations*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Modifies the neuron populations and replaces old population names with new ones in the transfer functions automatically.

    **Parameters:**

         + **populations** A dictionary containing neuron indices and is indexed by population names. Neuron indices could be defined by individual integers, lists of integers or python slices. Python slices are defined by a dictionary containing the from, to and step values.


**edit_state_machine**\ (\ *state_machine_name*, *state_machine*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
     Modify an existing State Machine by updating the script.

    **Parameters:**

        + **state_machine_name** A string containing the name of the state machine to be edited.
        + **state_machine** A string containing the new state machine code.


**edit_transfer_function**\ (\ *transfer_function_name*, *transfer_function*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Modify an existing Transfer Function by updating the script.

    **Parameters:**

        + **transfer_function_name** A string containing the name of the transfer function to be edited.
        + **transfer_function** A string containing the new transfer function code.


**get_brain**\ ( )
^^^^^^^^^^^^^^^^^^
    Gets the brain script.


**get_csv_data**\ (\ *file_name*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Returns the recorded csv data as a list, where each entry is a list of strings and represents one row in the original csv file.

    **Parameters:**

         + **file_name** The name of the csv file.


**get_populations**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^
    Gets the Neuron populations defined within the brain as a dictionary indexed by population names.


**get_state**\ ( )
^^^^^^^^^^^^^^^^^^
    Returns the current simulation state.


**get_state_machine**\ (\ *state_machine_name*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Gets the State Machine body for a given state machine name.

    **Parameters:**
      state_machine_name A string containing the name of the transfer function.


**get_transfer_function**\ (\ *transfer_function_name*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
      Gets the transfer function body for a given transfer function name.

      **Parameters:** transfer_function_name A string containing the name of the transfer function.


**launch**\ (\ *experiment_id*, *experiment_conf*, *server*, *reservation*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Attempt to launch and initialize the given experiment on the given servers. This should not be directly invoked by users, use the VirtualCoach interface to validate and launch a simulation.

    **Parameters:**

        + **experiment_id** A string representing the short name of the experiment to be launched (e.g. ExDTemplateHusky).
        + **experiment_conf** A string representing the configuration file for the experiment.
        + **server** A string representing the name of the server to try to launch on.
        + **reservation** A string representing a cluster resource reservation (if any).


**pause**\ ( )
^^^^^^^^^^^^^^
    Attempt to pause the simulation by transitioning to the paused state.


**print_csv_file_names**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Prints a list of all csv file names that contain recorded simulation data.


**print_state_machines**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Prints a list of the state-machine names defined in the experiment.


**print_transfer_functions**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Prints a list of the transfer-function names defined in the experiment.


**register_status_callback**\ (\ *callback*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Register a status message callback to be called whenever a simulation status message is received. This functionality is only available on installations with native ROS support.

    **Parameters:**

         + **callback** The callback function to be invoked.


**reset**\ (\ *reset_type*\ )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Resets the simulation according to the type the user wants. Successful reset will pause the simulation.

    **Parameters:**
         + **reset_type** The reset type the user wants to be performed. Possible values are full, robot_pose, world, brain. The possible reset types are stored in the config file.


**save_brain**\ ( )
^^^^^^^^^^^^^^^^^^^
    Saves the current brain script and populations to the storage


**save_csv**\ ( )
^^^^^^^^^^^^^^^^^
    Saves the recorded csv data to storage


**save_state_machines**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Saves the current state machines to the storage


**save_transfer_functions**\ ( )
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    Saves the current transfer functions to the storage


**save_world**\ ( )
^^^^^^^^^^^^^^^^^^^
    Saves the current sdf world to the storage


**start**\ ( )
^^^^^^^^^^^^^^
    Attempt to start the simulation by transitioning to the started state.


**stop**\ ( )
^^^^^^^^^^^^^
    Attempt to stop the simulation by transitioning to the stopped state.



