Usage of the NRP with NEST Desktop
==================================

When using the Neurorobotics Platform (NRP) in conjunction with the :term:`NEST` Simulator server, you have the option to utilize a powerful and user-friendly graphical user interface (GUI) known as `NEST Desktop <https://nest-desktop.readthedocs.io/en/latest/>`__. Here, both the NRP and NEST Desktop act as clients communicating with the NEST Simulator server.

.. note:: The integration of NRP with NEST Desktop is currently supported only through a specialized local Docker installation.

Installation
++++++++++++

Follow these steps to set up the environment for NRP with NEST Desktop:

#. **Install Docker and Docker Compose**: Start by installing Docker along with Docker Compose on your system. On Windows, make sure to install it natively and not inside WSL Ubuntu. 

    * Refer to the official `Docker installation guides <https://docs.docker.com/engine/install/>`__ for detailed instructions.
    * If you are on a Linux OS, consider `configuring Docker to run as a non-root user <https://docs.docker.com/engine/install/linux-postinstall/>`__. This eliminates the need to use **sudo** rights for every command.

#. **Download Docker Compose Configuration File**: Obtain the Docker Compose configuration file which contains settings for integrating NRP with NEST Desktop.

    * You can download the latest version from the NRP Bitbucket repository `here <https://bitbucket.org/hbpneurorobotics/user-scripts/src/development/docker-compose_nrp-nest-desktop-insite.yml>`__.

#. **Run Docker Compose**: Execute Docker Compose with the path to the configuration file you downloaded.

    ..  code-block:: bash

        docker compose -f docker-compose_nrp-nest-desktop-insite.yml up

    Docker will then download the necessary images and initiate the containers.

#. **Access the NRP Interface**: Open your web browser and navigate to `localhost:9200/#/esv-private <http://localhost:9200/#/esv-private>`__. Log in with the credentials :code:`nrpdemo:nestdesktop`.

Usage Example
+++++++++++++

Follow these steps for a smooth experiment using NRP with NEST Desktop:

1. Navigate to the NRP at `localhost:9200/#/esv-private <http://localhost:9200/#/esv-private>`__ and log in.

2. Select the *Templates* tab, then clone the **Holodeck Husky Braitenberg experiment with NEST-Desktop 3.2**.

    .. thumbnail:: nest-desktop-1.png
       :align: center
       :width: 100%

3. Under the *My Experiments* tab, select the cloned experiment and click the **Open NEST-Desktop** button.

4. In NEST Desktop settings, verify that the appropriate URLs for NEST and Insite are configured correctly.

    .. thumbnail:: nest-desktop-2.png
       :align: center
       :width: 100%

5. Download the NEST Desktop project :download:`configuration file <nest-desktop-project.json>` and import it into NEST Desktop.

6. Optionally, modify the neural network configuration. Ensure that any modifications are compatible with the experiment settings in NRP. When ready, select all stages except **Simulate** and press the **PREPARE** button to configure the NEST neurons.

    .. thumbnail:: nest-desktop-3.png
       :align: center
       :width: 100%

    .. note:: Configuring NEST should be completed prior to launching the simulation in the NRP.

7. Return to the NRP and initiate the simulation by pressing the **Launch** button. Observe the Husky Braitenberg vehicle's reaction to the red screen while NEST Desktop displays the spike activity.

8. To make further modifications to the network during simulation, first **Pause** the simulation in NRP. Then, alter the network in NEST Desktop and press the **PREPARE** button. Next, reload the brain configuration in NRP through the *Brain Editor* and resume the simulation.

    .. note:: Remember, network configuration in NEST Desktop should always be completed before launching or resuming the simulation in the NRP.

Happy experimenting! Utilize the combined power of the NRP and NEST Desktop for immersive neural network simulations.