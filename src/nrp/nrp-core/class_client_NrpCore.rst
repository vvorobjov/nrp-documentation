.. index:: pair: class; client::NrpCore
.. _doxid-classclient_1_1_nrp_core:

class client::NrpCore
=====================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	class NrpCore {
	public:
		// fields
	
		static int :target:`TIMEOUT_SEC<doxid-classclient_1_1_nrp_core_1ab29fc2ebf77d94562ca73f2437a0c6ba>` =  15;
		 :target:`child_pid<doxid-classclient_1_1_nrp_core_1a7a88e14ddba73a816674eb705fc82257>`;
		 :target:`stub<doxid-classclient_1_1_nrp_core_1ae2986dd93622c08e134698a7b6e636b1>`;

		// methods
	
		None :ref:`__init__<doxid-classclient_1_1_nrp_core_1a9f4fc7461c8282689c1a8c64896020ce>`(self self, address address, args args);
		None :ref:`__del__<doxid-classclient_1_1_nrp_core_1ac65027984997ca1f4d4ceca8d3a7d7ec>`(self self);
		None :ref:`kill_nrp_core_process<doxid-classclient_1_1_nrp_core_1a84cfc9e3f6493b91fa900e0977e72f72>`(self self);
		bool :ref:`wait_unit_server_ready<doxid-classclient_1_1_nrp_core_1a19099ae15f222397f3db2d56ae2a8db9>`(self self, channel channel);
		def :ref:`initialize<doxid-classclient_1_1_nrp_core_1abc683d9f01251ab75ea0cdca4333ad90>`(self self);
		def :ref:`runLoop<doxid-classclient_1_1_nrp_core_1a22e40044ee8f5e0856a5037f83fc5dc7>`(self self, num_iterations num_iterations);
		def :ref:`shutdown<doxid-classclient_1_1_nrp_core_1ad6242cde28c938c089ccdf909d318bc1>`(self self);
	};
.. _details-classclient_1_1_nrp_core:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; __init__
.. _doxid-classclient_1_1_nrp_core_1a9f4fc7461c8282689c1a8c64896020ce:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	None __init__(self self, address address, args args)

.. code-block:: cpp

	Spawns a child process for NRP Core with given arguments

.. index:: pair: function; __del__
.. _doxid-classclient_1_1_nrp_core_1ac65027984997ca1f4d4ceca8d3a7d7ec:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	None __del__(self self)

.. code-block:: cpp

	Deletes the NRP Core objects and kills its subprocess

.. index:: pair: function; kill_nrp_core_process
.. _doxid-classclient_1_1_nrp_core_1a84cfc9e3f6493b91fa900e0977e72f72:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	None kill_nrp_core_process(self self)

.. code-block:: cpp

	Sends SIGTERM signal to the NRP Core subprocess

.. index:: pair: function; wait_unit_server_ready
.. _doxid-classclient_1_1_nrp_core_1a19099ae15f222397f3db2d56ae2a8db9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool wait_unit_server_ready(self self, channel channel)

.. code-block:: cpp

	Waits for the gRPC server of the NRP Core process to start

.. index:: pair: function; initialize
.. _doxid-classclient_1_1_nrp_core_1abc683d9f01251ab75ea0cdca4333ad90:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	def initialize(self self)

.. code-block:: cpp

	Calls the init() RPC of the NRP Core Server

.. index:: pair: function; runLoop
.. _doxid-classclient_1_1_nrp_core_1a22e40044ee8f5e0856a5037f83fc5dc7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	def runLoop(self self, num_iterations num_iterations)

.. code-block:: cpp

	Calls the runLoop() RPC of the NRP Core Server

.. index:: pair: function; shutdown
.. _doxid-classclient_1_1_nrp_core_1ad6242cde28c938c089ccdf909d318bc1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	def shutdown(self self)

.. code-block:: cpp

	Calls the shutdown() RPC of the NRP Core Server

