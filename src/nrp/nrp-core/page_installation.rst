.. index:: pair: page; Installation Instructions
.. _doxid-installation:

Installation Instructions
=========================

The instructions required to install NRP-core from source (the only option currently available) are listed below.



.. _doxid-installation_1installation_requirements:

Requirements
~~~~~~~~~~~~

**WARNING:** Previous versions of the NRP install forked versions of several libraries, particularly NEST and Gazebo. Installing NRP-core in a system where a previous version of NRP is installed is known to cause conflicts. We strongly recommend not to do it.



.. _doxid-installation_1os:

Operative System
----------------

NRP-core has only been tested on **Ubuntu 20.04** at the moment, and therefore this OS and version are recommended. Installation in other environments might be possible but has not been tested yet.





.. _doxid-installation_1nest_version:

NEST
----

NRP-core only supports **NEST 3**.

As part of the installation process NEST 3 is built and installed. If you have an existing installation of NEST we recommend you to uninstall it before installing NRP-core. In case you still want to use your installed version, you can avoid the installation process to build and install NEST by changing the value of *ENABLE_NEST* from *FULL* to *CLIENT* in the root CMakeLists.txt file:

.. ref-code-block:: cpp

	set(ENABLE_NEST CLIENT)

In any case, be aware that NEST 2.x is incompatible with NRP-core.







.. _doxid-installation_1installation_dependencies:

Dependency Installation
~~~~~~~~~~~~~~~~~~~~~~~

.. ref-code-block:: cpp

	# Pistache REST Server
	sudo add-apt-repository ppa:pistache+team/unstable
	    
	# Gazebo
	sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
	wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
	    
	sudo apt update
	sudo apt install git cmake libpistache-dev g++-10 libboost-python-dev libboost-filesystem-dev libboost-numpy-dev libcurl4-openssl-dev nlohmann-json3-dev libzip-dev cython3 python3-numpy libgrpc++-dev protobuf-compiler-grpc libprotobuf-dev doxygen libgsl-dev libopencv-dev python3-opencv python3-pil python3-pip
	
	# required by gazebo engine
	sudo apt install libgazebo11-dev gazebo11 gazebo11-plugin-base
	
	# required by nest-server (which is built and installed along with nrp-core)
	sudo apt install python3-flask python3-flask-cors python3-restrictedpython uwsgi-core uwsgi-plugin-python3 
	
	# required by nrp-server, which uses gRPC python bindings and mpi
	pip install grpcio-tools pytest docopt mpi4py
	   
	# ROS
	Install ROS: follow the installation instructions: http://wiki.ros.org/noetic/Installation/Ubuntu. To enable ros support in nrp on `ros-noetic-ros-base` is required.
	
	Tell nrp-core where your catkin workspace is located: export a variable CATKIN_WS pointing to an exisiting catking workspace root folder. If the variable does not exist, a new catkin workspace will be created at `${HOME}/catkin_ws`.
	    
	# Fix deprecated type in OGRE (std::allocator<void>::const_pointer has been deprecated with glibc-10). Until the upstream libs are updated, use this workaround. It changes nothing, the types are the same
	sudo sed -i "s/typename std::allocator<void>::const_pointer/const void*/g" /usr/include/OGRE/OgreMemorySTLAllocator.h





.. _doxid-installation_1installation_procedure:

Installation
~~~~~~~~~~~~

.. ref-code-block:: cpp





.. _doxid-installation_1installation_environment:

Setting the enviroment
~~~~~~~~~~~~~~~~~~~~~~

In order to properly set the environment to run experiments with NRP-core, please make sure to add the lines below to your ~/.bashrc file

.. ref-code-block:: cpp





.. _doxid-installation_1installation_opensim:

Special steps for installing OpenSim
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Installation of the OpenSim engine requires some modification over the instructions found at `https://github.com/opensim-org/opensim-core <https://github.com/opensim-org/opensim-core>`__. The procedure below should therefore be followed.

.. ref-code-block:: cpp

	sudo apt-get update
	# For ipopt
	sudo apt-get install -y libblas-dev libatlas-base-dev
	sudo apt-get install -y gcc g++ gfortran patch libmetis-dev
	sudo apt-get install -y coinor-libipopt-dev
	# For adolc
	sudo apt-get install -y libtool libtool-bin
	sudo apt-get install -y autoconf
	sudo apt-get install -y libadolc-dev
	
	sudo apt-get --yes install  cmake cmake-curses-gui \
	                           freeglut3-dev libxi-dev libxmu-dev \
	                           liblapack-dev swig python-dev \
	                           openjdk-8-jdk
	export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
	
	# Install PATH
	OPENSIM_PATH=$HOME/Documents/OpenSim
	mkdir -p $OPENSIM_PATH
	cd $OPENSIM_PATH
	
	# Build Opensim dependencies
	git clone https://github.com/opensim-org/opensim-core.git
	mkdir opensim_dependencies_build
	cd opensim_dependencies_build
	cmake ../opensim-core/dependencies/ \
	      -DCMAKE_INSTALL_PREFIX='../opensim_dependencies_install' \
	      -DCMAKE_BUILD_TYPE=RelWithDebInfo
	make -j4
	
	# Environments for opensim
	sudo alias python=python3
	sudo apt-get install -y python3-pip
	sudo pip3 install numpy    
	
	cd $OPENSIM_PATH
	mkdir opensim_build
	cd opensim_build
	JAVA_TOOL_OPTIONS=-Dfile.encoding=UTF8
	cmake ../opensim-core \
	      -DCMAKE_INSTALL_PREFIX="../opensim_install" \
	      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
	      -DOPENSIM_DEPENDENCIES_DIR="../opensim_dependencies_install" \
	      -DBUILD_PYTHON_WRAPPING=ON \
	      -DBUILD_JAVA_WRAPPING=ON \
	      -DWITH_BTK=ON
	make -j4
	make -j4 install
	
	cd $OPENSIM_PATH
	cd opensim_install/lib/python3.8/site-packages
	python3 setup.py install
	
	cd $HOME
	echo 'export LD_LIBRARY_PATH='$OPENSIM_PATH'/opensim_install/lib:$LD_LIBRARY_PATH' >> $HOME/.bashrc
	echo 'export PYTHONPATH='$OPENSIM_PATH'/opensim_install/lib/python3.8/site-packages:$PYTHONPATH' >> $HOME/.bashrc
	source $HOME/.bashrc

