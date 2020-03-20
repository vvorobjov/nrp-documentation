Tutorial: Launching an Experiment from the Virtual Coach
========================================================

Starting the Virtual Coach
^^^^^^^^^^^^^^^^^^^^^^^^^^
There are different ways to start a Virtual Coach instance. With a local NRP install you can use the alias *cle-virtual-coach* in three different ways:

1. **Launch a Jupyter Notebook session**

   To launch a Jupyter Notebook session just run `cle-virtual-coach jupyter notebook` in a terminal. Of course, Jupyter Notebook has to be installed prior.

2. **Launch an interactive python interpreter session**

   To launch an interactive python interpreter session just run `cle-virtual-coach python` in a terminal.

3. **Launch a python script with arguments**

   To launch a python script `foo.py` with arguments `a b c` just run `cle-virtual-coach foo.py a b c`.

This information is also available in the alias help cle-virtual-coach -h.

Launching a Simulation
^^^^^^^^^^^^^^^^^^^^^^
Let's assume we launched the Virtual Coach in an interactive session, whether in a jupyter notebook or in the python interpreter. The first thing we need to do is to import the Virtual Coach and create a VirtualCoach instance with the appropriate backend.

.. code-block:: python

    from hbp_nrp_virtual_coach.virtual_coach import VirtualCoach
    vc = VirtualCoach(environment='local', storage_username='nrpuser')

Here we chose the local machine as the backend server. You can choose custom backends by modifying proxy section in the config.json. The default environment options are `staging`, `dev` and `local`.

Once we created a VirtualCoach instance, we can use it to check out the current state of our environment. We can see a list of the available experiments to run, a list of the available servers to run experiments on, and a list of the currently running experiments.

.. code-block:: python

    vc.print_templates()
    vc.print_available_servers()
    vc.print_running_experiments()

After making sure our experiments exist and enough resources are available, we can attempt to launch an experiment. In this next section we will launch the Template Husky experiment. To launch a specific experiment, we need to specify its configuration name, which we can get from the experiment list.

.. code-block:: python

    sim = vc.launch_experiment('ExDTemplateHusky')

Launching an experiment can take some time and once it's been successfully launched, we'll get a `Simulation Successfully Created` log.

We can also make sure that the experiment has been launched by querying the Virtual Coach for currently running experiments.
