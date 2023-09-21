..  _source-installation:

=========================
Installation from Source
=========================

.. note:: Currently, only Ubuntu 20.04 is officially supported for the NRP installation from source.

Pre-installation Steps
----------------------

1. **Set the HBP folder**:

.. code-block:: bash

    export HBP=~/NRP4

Add this variable to your `.bashrc` for persistence:

.. code-block:: bash

    echo "export HBP=~/NRP4" >> ~/.bashrc

2. **Ensure the HBP folder exists**:

.. code-block:: bash

    mkdir -p "${HBP}"


3. **Install Nginx and HAProxy**:

.. code-block:: bash

    sudo apt-get install nginx-extras lua-cjson haproxy
    sudo systemctl disable haproxy.service
    sudo service haproxy stop
    sudo service nginx stop

4. **Install nvm and Node** versions 8 & 14:

.. code-block:: bash

    curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.3/install.sh | bash
    source ~/.bashrc
    nvm install 8
    nvm install 14


5. **Clone** `nrp-user-scripts` repository (for the last stable version use `master` branch, for the development version use `development` branch):

.. code-block:: bash

    cd "${HBP}"
    git clone -b master https://bitbucket.org/hbpneurorobotics/nrp-user-scripts.git

6. **Clone** `nrp-backend`, `nrp-proxy`, `nrp-frontend`, `nrp-core` repositories using `nrp-user-scripts` or do it manually:

.. code-block:: bash

    cd ${HBP}/nrp-user-scripts
    ./clone-all-repos

7. **Install NRP-Core**. Refer to the :doc:`NRP-Core Installation instructions <nrp-core/page_installation>`.

.. warning:: The proper installation of the NRP-Core is essential for the functioning of the NRP source installation.


Configuration and Building
--------------------------

8. **Add required paths to PYTHONPATH**:

.. code-block:: bash

    export PYTHONPATH="${NRP_INSTALL_DIR}"/lib/python3.8/site-packages:"${PYTHONPATH}"
    export PYTHONPATH="${NRP_DEPS_INSTALL_DIR}"/lib/python3.8/site-packages:"${PYTHONPATH}"

For persistence, you can also add it to `.bashrc`.

9. **Clean old storage** or set a non-default `STORAGE_PATH` in `${HBP}/nrp-user-scripts/nrp_variables`.

10. **Source** `nrp_variables` from `nrp-user-scripts`:

.. code-block:: bash

    cd "${HBP}"/nrp-user-scripts
    source nrp_variables

.. warning:: Temporarily remove it when the need of rebuilding nrp-core arises (e.g. when developing the latter).

11. **Configure NRP**:

.. code-block:: bash

    ./configure_nrp


12. **Update NRP with build option**:

.. code-block:: bash

    ./update_nrp build

13. **Add default user to the database** (if not added before):

.. code-block:: bash

    bash ./configure_storage_database

14. **Source** `nrp_aliases`:

.. code-block:: bash

    source nrp_aliases

15. **Run NRP reverse proxies**:

.. code-block:: bash

    nrp-reverse-proxies

16. **Start the MQTT broker** (using the mosquitto Docker broker or another of your choice, ensuring the proper ports are open):

.. code-block:: bash

    docker run -d -p 1883:1883 -p 9001:9001 -p 8883:8883 -v $HBP/nrp-user-scripts/config_files/mosquitto/mosquitto.conf:/mosquitto/config/mosquitto.conf eclipse-mosquitto

17. **Start NRP**:

.. code-block:: bash

    nrp-start


Access
------

- Once everything is set up, the frontend should be accessible at `http://localhost:9000`.
