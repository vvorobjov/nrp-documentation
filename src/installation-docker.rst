..  _docker-installation:

Installation with Docker
========================

NRP can be executed using Docker on both Ubuntu and Windows.

..  note:: **WSL**: If you are using Windows, ensure that the `Windows Subsystem for Linux <https://learn.microsoft.com/en-us/windows/wsl/install>`_ (WSL) is properly installed and set up.

..  note:: **Docker engine**: To run NRP in Docker containers, the `Docker engine <https://docs.docker.com/engine/install/>`_ must be installed and configured on your system. Additionally, it's recommended to allow `management of Docker as a non-root user <https://docs.docker.com/engine/install/linux-postinstall/>`_. If not, all Docker commands must be prefixed with :code:`sudo`.

..  note:: Alongside the Docker engine, the Docker Compose plugin is required. Refer to the `installation guide <https://docs.docker.com/compose/install/>`_ for details.

Steps:

1. **Verify Docker and Docker Compose installation**:

    ..  code-block:: shell

        docker version
        docker compose version

2. **Set the HBP folder**:

    ..  code-block:: shell

        export HBP=~/NRP4

    To persist this setting across sessions, add the environment variable to your :code:`~/.bashrc`: 

    ..  code-block:: shell
        
        echo "export HBP=~/NRP4" >> ~/.bashrc

3. **Ensure the $HBP folder exists**:

    ..  code-block:: shell

        mkdir -p "${HBP}"

4. **Clone** `nrp-user-scripts` repository (for the last stable version use `master` branch, for the development version use `development` branch):

    ..  code-block:: shell

        cd "${HBP}"
        git clone -b master https://bitbucket.org/hbpneurorobotics/nrp-user-scripts.git

..  note:: Ensure the following port is available:

    * 9000 (for haproxy-service)

5. **Clean or set the STORAGE_PATH**:

    ..  code-block:: shell

        export STORAGE_PATH=~/nrpStorage_docker
        echo "export STORAGE_PATH=~/nrpStorage_docker" >> ~/.bashrc

6. **Execute the docker-compose script**:

    ..  code-block:: shell

        cd "${HBP}"/nrp-user-scripts
        ./start_nrp_docker.sh # accepts any docker-compose parameter e.g. "-d" for daemon mode

Once the setup is complete, access the frontend at `http://localhost:9000`.

..  note:: The default Docker configuration uses the :code:`nest-gazebo` backend image variant, which bundles Gazebo and NEST and is required by most simulation templates. To select a different variant, set the :code:`NRP_BACKEND_TAG` environment variable **before** running :code:`start_nrp_docker.sh` — there is no need to edit the compose file:

    ..  code-block:: shell

        # nest-gazebo (default): Gazebo + NEST, needed by most templates
        # vanilla: slim image without simulators (only the exchange_tf template is compatible)
        export NRP_BACKEND_TAG=vanilla

        cd "${HBP}"/nrp-user-scripts
        ./start_nrp_docker.sh

    :code:`NRP_BACKEND_TAG` accepts :code:`nest-gazebo` (default) or :code:`vanilla`, and its value is used verbatim as the published backend image tag :code:`hbpneurorobotics/nrp-backend:<tag>` (i.e. :code:`hbpneurorobotics/nrp-backend:nest-gazebo` or :code:`hbpneurorobotics/nrp-backend:vanilla`). When it is left unset, :code:`start_nrp_docker.sh` defaults it to :code:`nest-gazebo`, so the export is only needed to switch to the slim :code:`vanilla` image.
