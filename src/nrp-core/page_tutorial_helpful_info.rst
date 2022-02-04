.. index:: pair: page; Helpful information
.. _doxid-tutorial_helpful_info:

Helpful information
===================



.. _doxid-tutorial_helpful_info_1tutorial_helpful_info_husky:

Additional Models for Braitenberg Husky experiments
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The Husky examples, examples/husky_braitenberg and examples/husky_braitenberg_nest_server, require additional gazebo models that are not included in the NRP-core repository. These assets can be found in other NRP repositories, and the instructions below show how to download and make them available to gazebo.

.. ref-code-block:: cpp

	# Create a new directory, for example in your home directory:
	mkdir ${HOME}/nrp
	cd ${HOME}/nrp
	
	# Clone our repositories that contain the necessary assets:
	git clone https://@bitbucket.org/hbpneurorobotics/models.git
	git clone https://@bitbucket.org/hbpneurorobotics/gzweb.git
	
	#Export your NRP-core installation directory as HBP. This variable will be needed to create symlinks to gazebo models
	export HBP=/home/${USER}/.local/nrp
	
	# Create necessary directories for gazebo models and gzweb assets:
	mkdir -p ${HBP}/gzweb/http/client/assets
	mkdir ${HOME}/.gazebo/models
	
	# Run the create-symlinks.sh script from the models repository. This will create necessary symlinks 
	# to the models in .gazebo/models, and to the assets in ${HBP}/gzweb/http/client/assets
	
	cd models
	./create-symlinks.sh

Now it should be possible to run the Husky experiments!

