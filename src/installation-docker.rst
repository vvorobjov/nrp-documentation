

Installation with Docker
========================

NPR can be run in Docker in Ubuntu and Windows.

..  note:: **WSL**. If you are usin Windows, make sure that the `Windows Subsystm Linux <https://learn.microsoft.com/en-us/windows/wsl/install>`_ (WSL) is installed and configured.

..  note:: **Docker engine**. In order to run the NRP in Docker containers, the `Docker engine <https://docs.docker.com/engine/install/>`_ and docker-compose should be installed and configured in your system. Moreover, we recommend to allow `managing Docker as a non-root user <https://docs.docker.com/engine/install/linux-postinstall/>`_. Otherwise, you will need to preface all Docker commands with :code:`sudo`.

*   set HBP folder, i.e.

    ..  code-block:: shell

        export HBP=~/NRP4

    you can add this variable to your :code:`~/.bashrc` so that it is always available: 

    ..  code-block:: shell
        
        echo "export HBP=~/NRP4" >> ~/.bashrc


*   make sure, that the $HBP folder exists

    ..  code-block:: shell

        mkdir -p "${HBP}"

*   clone/checkout there nrp-user-scripts repository 

    ..  code-block:: shell

        cd "${HBP}"
        git clone -b master https://bitbucket.org/hbpneurorobotics/nrp-user-scripts.git



..  note:: The following port must be available:

    *   9000 (haproxy-service)

*   clean old :code:`~/.opt/nrpStorage` (or set non-default :code:`STORAGE_PATH`)

    ..  code-block:: shell

        export STORAGE_PATH=~/nrpStorage_docker
        echo "export STORAGE_PATH=~/nrpStorage_docker" >> ~/.bashrc

*   run the **docker-compose** with the script

    ..  code-block:: shell

        cd "${HBP}"/nrp-user-scripts
        ./start_nrp_docker.sh # accepts any docker compose parameter e.g. "-d" daemon mode

*   the frontend should be available at http://localhost:9000

..  note:: The default composition of the Docker installation is aimed for using NRP with Gazebo and NEST. In order to use opensim and TVB, the other backend image should be used, see service **nrp-backend-service** in :code:`"${HBP}"/nrp-user-scripts/docker-compose.yaml`

    ..  code-block:: shell

        # image: docker-registry.ebrains.eu/nrp/nrp-core/backend-nrp-opensim-tvb-ubuntu20${NRP_CORE_TAG}
        image: docker-registry.ebrains.eu/nrp/nrp-core/backend-nrp-gazebo-nest-ubuntu20${NRP_CORE_TAG}

    Uncommenting the first image and commenting the second will switch the images. The other possible image is **backend-nrp-vanilla-ubuntu20**, which doesn't have neither Gazebo nor opensim. The only working template there is *exchange_tf*.
