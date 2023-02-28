..  _source-installation:

Installation from source
========================

..  note:: Currently, only Ubuntu 20.04 is officially supported for the NRP installation from source.


*   Set HBP folder, i.e.

    ..  code-block:: shell

        export HBP=~/NRP4

    You can add this variable to your :code:`~/.bashrc` so that it is always available: 

    ..  code-block:: shell

        echo "export HBP=~/NRP4" >> ~/.bashrc


*   Make sure, that the $HBP folder exists

    ..  code-block:: shell

        mkdir -p "${HBP}"

*   clone/checkout there nrp-user-scripts repository 

    ..  code-block:: shell

        cd "${HBP}"
        git clone -b master https://bitbucket.org/hbpneurorobotics/nrp-user-scripts.git



*   clone/checkout **nrp-backend**, **nrp-proxy**, **nrp-frontend**, **nrp-core** repositories. This can be done with help of script from **nrp-user-scripts** (or manually)

    ..  code-block:: shell

        cd ${HBP}/nrp-user-scripts
        ./clone-all-repos

*   install **nrp-core** as usual and as desired, see :doc:`NRP-Core Installation instructions <nrp-core/page_installation>`.

*   add :code:`${NRP_INSTALL_DIR}/lib/python3.8/site-packages` and :code:`${NRP_DEPS_INSTALL_DIR}/lib/python3.8/site-packages` to :code:`PYTHONPATH`

    ..  code-block:: shell

        export PYTHONPATH="${NRP_INSTALL_DIR}"/lib/python3.8/site-packages:"${PYTHONPATH}"
        export PYTHONPATH="${NRP_DEPS_INSTALL_DIR}"/lib/python3.8/site-packages:"${PYTHONPATH}"

    You can also add it to :code:`~/.bashrc`.

*   clean old :code:`~/.opt/nrpStorage` (or set non-default :code:`STORAGE_PATH` in :code:`${HBP}/nrp-user-scripts/nrp_variables`)

*   install nvm and then two versions of node 8 & 14

    ..  code-block:: shell

        curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash
        source ~/.bashrc
        nvm install 8
        nvm install 14

*   in **nrp-user-scripts** source :code:`nrp_variables`

    ..  code-block:: shell

        cd "${HBP}"/nrp-user-scripts
        source nrp_variables

*   run :code:`./configure_nrp`

    ..  code-block:: shell

        cd "${HBP}"/nrp-user-scripts
        ./configure_nrp

*   run :code:`./update_nrp build`

    ..  code-block:: shell

        cd "${HBP}"/nrp-user-scripts
        ./update_nrp build

*   run :code:`./configure_nrp`

*   source :code:`nrp_aliases`

    ..  code-block:: shell

        cd "${HBP}"/nrp-user-scripts
        source nrp_aliases

*   run nrp-reverse-proxies

    ..  code-block:: shell

        nrp-reverse-proxies

*   start MQTT broker. You can use mosquitto Docker broker, or any other on your choice (but make sure the proper ports are open)

    ..  code-block:: shell

        docker run -d -p 1883:1883 -p 9001:9001 -p 8883:8883 -v $HBP/nrp-user-scripts/config_files/mosquitto/mosquitto.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto

*   run :code:`nrp-start`

    ..  code-block:: shell

        nrp-start

*   The frontend should be available at http://localhost:9000
