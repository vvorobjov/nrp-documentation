.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>
   
Usage of the NRP with NEST Desktop
==================================

When using the Neurorobotics Platform (NRP) in conjunction with the :term:`NEST Server`, you have the option to utilize a powerful and user-friendly graphical user interface (GUI) known as `NEST Desktop <https://nest-desktop.readthedocs.io/en/latest/>`__. Here, both the NRP and NEST Desktop act as clients communicating with the NEST Simulator server.

.. note:: The integration of NRP with NEST Desktop is currently supported only through a specialized local Docker installation with the use of Docker Compose.

Installation
++++++++++++

Follow the steps described in :ref:`docker-installation` up to "Running the NRP" (do not run :code:`./start_nrp_docker.sh` script).

#. **Run NRP with NEST Desktop**: Execute the script starting Docker Compose with NRP and NEST Desktop.

    ..  code-block:: bash

        cd "${HBP}"/nrp-user-scripts
        ./nrp_nest_desktop.sh # accepts any docker compose parameter e.g. "-d" daemon mode

    Docker will then download the necessary images and initiate the containers.

#. **Access the NRP Interface**: Open your web browser and navigate to `localhost:9000 <http://localhost:9000>`__. Log in with the credentials :code:`nrpuser:password`.

Usage Example
+++++++++++++

Follow these steps for a smooth experiment using NRP with NEST Desktop:

1. Navigate to the NRP at `localhost:9000 <http://localhost:9000>`__ and log in.

2. Select the *Templates* tab, then clone the **husky_simulation_nest_server**.

    .. thumbnail:: nest-desktop-1.png
       :align: center
       :width: 100%

3. Under the *My Experiments* tab, select the cloned experiment and click the **Open** button.

    .. thumbnail:: nest-desktop-2.png
       :align: center
       :width: 100%

4. In the NRP experiment workbench, find the NEST Desktop icon in the left panel and drag the tool in the working area..

    .. thumbnail:: nest-desktop-3.png
       :align: center
       :width: 100%

    .. thumbnail:: nest-desktop-4.png
       :align: center
       :width: 100%

5. Download the NEST Desktop project :download:`configuration file <nest-desktop-project.json>` and import it into NEST Desktop

6. Optionally, modify the neural network configuration. Ensure that any modifications are compatible with the experiment settings in NRP. When ready, select all stages except **Simulate** and press the **PREPARE** button to configure the NEST neurons. The NEST Desktop will become ready to receive spikes signals.

7. Press the **Launch** button to initialize the simulation in the NRP. The status bar should become yellow when the simulation is ready to be started.

    .. thumbnail:: nest-desktop-5.png
       :align: center
       :width: 100%

    .. note:: Configuring NEST should be completed prior to launching the simulation in the NRP.


8. Press the **Start** button to run the simulation and observe the spikes activity in the NEST Desktop.

    .. thumbnail:: nest-desktop-6.png
       :align: center
       :width: 100%
       

    .. note:: Remember, network configuration in NEST Desktop should always be completed before launching in the NRP.

Happy experimenting! Utilize the combined power of the NRP and NEST Desktop for immersive neural network simulations.
