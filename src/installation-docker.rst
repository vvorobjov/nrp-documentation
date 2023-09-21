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

..  note:: The default Docker configuration is optimized for using NRP with Gazebo and NEST. To utilize opensim and TVB, a different backend image is necessary. Refer to the **nrp-backend-service** in :code:`"${HBP}"/nrp-user-scripts/docker-compose.yaml`. 

    ..  code-block:: shell

        # image: docker-registry.ebrains.eu/nrp/nrp-core/backend-nrp-opensim-tvb-ubuntu20${NRP_IMAGE_TAG}
        image: docker-registry.ebrains.eu/nrp/nrp-core/backend-nrp-gazebo-nest-ubuntu20${NRP_IMAGE_TAG}

    To switch between images, uncomment the desired image and comment out the other. Another option is the **backend-nrp-vanilla-ubuntu20** image, which excludes both Gazebo and opensim. The only compatible template with this is *exchange_tf*.
