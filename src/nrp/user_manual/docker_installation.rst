.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>
    
.. _docker-installation:

Installation and running NRP using Docker containers
====================================================

.. seealso::
    :ref:`Source Installation <source-installation>`

This installation guide is for users who want to try out the :abbr:`NRP (Neurorobotics Platform)` locally on their machine, clone template experiments, or create new ones based on our template models. More expert users or developers who might want to change code directly in the :term:`Platform` or add Gazebo plugins and robot controllers should install the :abbr:`NRP (Neurorobotics Platform)` :ref:`from source<source-installation>` instead.

This installation uses :term:`Docker` as the underlying package manager. It is technically possible for users to open a shell in the containers and hack them, but they must be aware that containers are replaced on Platform updates and all there changes inside containers will be lost. Of course, all user data, custom experiments and models are safe and kept between updates!

.. note:: It is our wish to provide an open and inclusive access to our platform, therefore users can freely download and install the :abbr:`NRP (Neurorobotics Platform)` on their machines. However, we would like to remind all the :abbr:`NRP (Neurorobotics Platform)` current and prospective users of our commitment to the EC `Horizon 2020`_ guidelines and recommendations which regulate the use of publicly funded research and research applications for benign use only. You can read more about this in the `HBP Opinion`_ on ‘Responsible Dual Use'.


.. _HBP Opinion: https://www.humanbrainproject.eu/en/follow-hbp/news/opinion-on-responsible-dual-use-from-the-human-brain-project/
.. _Horizon 2020: https://ec.europa.eu/programmes/horizon2020/

.. contents:: Guide Contents
    :depth: 3


..  warning:: Usage of low-bandwidth networks.

    This install procedure involves downloading of several GBs from the Internet. Public WiFi or slow connections will take a long time or possibly fail.


.. _preliminary step:

Preliminary step
++++++++++++++++++

Microsoft Windows
~~~~~~~~~~~~~~~~~
..  note:: Linux/Mac users, skip this part.

#. Windows users have to first install :abbr:`WLS (Windows Linux Subsystem)` 2 and a brand of Linux on top of it from the Windows Store (we tested WSL with Ubuntu 18.04 and Ubuntu 20.04). See `here for instructions <https://docs.microsoft.com/en-us/windows/wsl/install-win10>`__.
#. Open an Linux terminal from the Windows start menu (in our case an Ubuntu menu item) and proceed with the next step.

macOS
~~~~~
..  note:: Linux/Windows users, skip this part.

..  note:: on macOS only Docker versions >= 18.03 are supported.

#. Add an alias into your hosts file::

    echo "127.0.0.1 host.docker.internal" | sudo tee -a /etc/hosts

#. Restart your computer.

Installation
+++++++++++++++
This install procedure should work on any Linux OS, macOS (with Docker version >=18.03) and on Windows 10 if you did the :ref:`preparatory part<preliminary step>`. Other operating systems have not been tested and it is likely they will not work with this bash script.

#. The first step is to install Docker on your system (on Windows, install it in Windows, not in the WSL Ubuntu).
#. Then `download the following script`_ and run it in a terminal (Linux terminal for Windows users). The script should auto-update itself later when needed.
#. The following steps will install and configure the Docker images of the :abbr:`NRP (Neurorobotics Platform)` on your system. Your user data will be stored in Docker volumes. So everything is cleanly self contained. You just need to specify the release type of the NRP.

  ..  code-block:: bash

      chmod 755 nrp_installer.sh 
      ./nrp_installer.sh install VERSION
      # Option 1: ./nrp_installer.sh install latest 
      # Option 2: ./nrp_installer.sh install legacy

In order to install the latest release of the :abbr:`NRP (Neurorobotics Platform)` (Python 3.8 compatible) use :code:`latest` as VERSION. In case the VERSION parameter is set to :code:`legacy`, then the script installs the legacy release of the :abbr:`NRP (Neurorobotics Platform)` (v3.0.5 - Python 2.7 compatible). Note, that **omitting VERSION** parameter forces the script to install **the latest release** of the NRP. You can run the script without any arguments to get a list of other options, e.g. uninstall, restart, connect to containers (open a bash inside them) etc.

.. _download the following script: https://neurorobotics-files.net/index.php/s/83zqkdp5PXQXMzz/download

Postliminary steps
+++++++++++++++++++++++

Installing SpiNNaker
~~~~~~~~~~~~~~~~~~~~

.. note:: This step might only be necessary for users who own a SpiNNaker board and want to connect this board with the NRP.

By default, the :abbr:`NRP (Neurorobotics Platform)` works with a 4-chip SpiNNaker board using the standard IP-address as shown here. To change the standard settings, please follow these steps:

..  code-block:: bash

    ./nrp_installer.sh connect_backend
    # Open a new terminal and enter
    nano ~/.spynnaker.cfg
    # Modify the IP-address and board settings in the text file 

More information on the SpiNNaker settings can be found in the `SpiNNaker documentation`_.

..  code-block:: bash

    # Back in the old terminal enter
    ./nrp_installer.sh restart

.. _SpiNNaker documentation: http://spinnakermanchester.github.io/spynnaker/5.0.0/PyNNOnSpinnakerInstall.html

Connect to the Neurorobotics Platform
+++++++++++++++++++++++++++++++++++++++++++++

After running the installation script, in order to connect to the platform open your browser (Firefox and Chrome are officially supported) and go to the following link: http://localhost:9000/#/esv-private .

You will be prompted to enter a username and password. On local installs like this one, there is a default user "**nrpuser**" and password "**password**". Creating new users is currently not possible from the user interface.

Troubleshooting
+++++++++++++++++++++++++++

Docker network
~~~~~~~~~~~~~~
If there are troubles setting up the Docker network, you can try a different subnet by changing it in the installer script.

Windows issues
~~~~~~~~~~~~~~
In Windows install, communication with the Docker daemon has been reported. In that case, follow this:

* check the setting :command:`expose deamon on tcp://localhost:2375` in Docker;
* in Ubuntu add the line at the bottom of your :code:`$HOME/.bashrc` (use the nano editor, for example)::

    export DOCKER_HOST=tcp://localhost:2375 
