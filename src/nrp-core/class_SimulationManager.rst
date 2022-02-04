.. index:: pair: class; SimulationManager
.. _doxid-class_simulation_manager:

class SimulationManager
=======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Main NRP server class. Manages simulation execution, loads plugins, and creates server if requested. :ref:`More...<details-class_simulation_manager>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <simulation_manager.h>
	
	class SimulationManager: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// typedefs
	
		typedef std::mutex :target:`sim_mutex_t<doxid-class_simulation_manager_1a16a00820d6d3220fd03ca2d941f20dde>`;
		typedef std::unique_lock<:ref:`sim_mutex_t<doxid-class_simulation_manager_1a16a00820d6d3220fd03ca2d941f20dde>`> :target:`sim_lock_t<doxid-class_simulation_manager_1adc24a19cb3a6911979d483aa32a7970a>`;

		// construction
	
		:ref:`SimulationManager<doxid-class_simulation_manager_1af976d90860cfe9be1ebcc3a9423b3ddd>`(const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simulationConfig);

		// methods
	
		:ref:`FTILoopConstSharedPtr<doxid-fti__loop_8h_1aee94d67e4efd86acf417f306ebf1e766>` :ref:`simulationLoop<doxid-class_simulation_manager_1a95015f9f49a3d525d5ce32e753ccbd58>`() const;
		:ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>` :ref:`simulationConfig<doxid-class_simulation_manager_1ad4b28f92ed9dd7147e3ac05ca16eded2>`();
		:ref:`jsonConstSharedPtr<doxid-json__schema__utils_8h_1aefb483b68c65a60571e42f7b3111ddfd>` :ref:`simulationConfig<doxid-class_simulation_manager_1a9bccde2214a1edbe58d57958dcab10be>`() const;
	
		void :ref:`initFTILoop<doxid-class_simulation_manager_1a34e9ae849be6d01c1df8a39d2e61bbbf>`(
			const :ref:`EngineLauncherManagerConstSharedPtr<doxid-engine__launcher__manager_8h_1af0948ce319f25e479423423640c94c0c>`& engineLauncherManager,
			const :ref:`MainProcessLauncherManager::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>`& processLauncherManager
		);
	
		bool :ref:`resetSimulation<doxid-class_simulation_manager_1a4d733b33b59c886ac13f572ee25e82e1>`();
		bool :ref:`runSimulationUntilTimeout<doxid-class_simulation_manager_1a844a575a84cc9e3aa55b632cb12f3851>`(int frac = 1);
		void :ref:`runSimulation<doxid-class_simulation_manager_1a561a1f5a5f0bee6f4a129bf3c6f0cc7a>`(unsigned numIterations);
		void :ref:`runSimulationOnce<doxid-class_simulation_manager_1a05d7628dfcf057f42bd4f0d041f5a116>`();
		void :ref:`shutdownLoop<doxid-class_simulation_manager_1a82bf87514e9c01fe615912b427e971bd>`();
		static :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>` :ref:`configFromParams<doxid-class_simulation_manager_1aec148802afb88105e09b2a7a474ab944>`(const cxxopts::ParseResult& args);
		static SimulationManager :ref:`createFromConfig<doxid-class_simulation_manager_1abc5fe2e0b3b3bd0718d3281637d09320>`(:ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& config);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// typedefs
	
		typedef std::shared_ptr<T> :ref:`shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`;
		typedef std::shared_ptr<const T> :ref:`const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>`;
		typedef std::unique_ptr<T> :ref:`unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`;
		typedef std::unique_ptr<const T> :ref:`const_unique_ptr<doxid-class_ptr_templates_1aef0eb44f9c386dbf0de54d0f5afac667>`;

.. _details-class_simulation_manager:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Main NRP server class. Manages simulation execution, loads plugins, and creates server if requested.

Construction
------------

.. index:: pair: function; SimulationManager
.. _doxid-class_simulation_manager_1af976d90860cfe9be1ebcc3a9423b3ddd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	SimulationManager(const :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& simulationConfig)

Constructor.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- serverConfig

		- Server configuration

	*
		- simulationConfig

		- Simulation configuration

Methods
-------

.. index:: pair: function; simulationLoop
.. _doxid-class_simulation_manager_1a95015f9f49a3d525d5ce32e753ccbd58:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`FTILoopConstSharedPtr<doxid-fti__loop_8h_1aee94d67e4efd86acf417f306ebf1e766>` simulationLoop() const

Get simulation loop.



.. rubric:: Returns:

Returns pointer to simulation loop. If no loop is loaded, return nullptr

.. index:: pair: function; simulationConfig
.. _doxid-class_simulation_manager_1ad4b28f92ed9dd7147e3ac05ca16eded2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>` simulationConfig()

Get simulation config.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- simLock

		- Pass simulation lock if already owned



.. rubric:: Returns:

Returns pointer to simulation config as well as simulation lock. If no config is loaded, return nullptr

.. index:: pair: function; simulationConfig
.. _doxid-class_simulation_manager_1a9bccde2214a1edbe58d57958dcab10be:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`jsonConstSharedPtr<doxid-json__schema__utils_8h_1aefb483b68c65a60571e42f7b3111ddfd>` simulationConfig() const

Get simulation config.



.. rubric:: Returns:

Returns pointer to simulation config. If no config is loaded, return nullptr

.. index:: pair: function; initFTILoop
.. _doxid-class_simulation_manager_1a34e9ae849be6d01c1df8a39d2e61bbbf:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void initFTILoop(
		const :ref:`EngineLauncherManagerConstSharedPtr<doxid-engine__launcher__manager_8h_1af0948ce319f25e479423423640c94c0c>`& engineLauncherManager,
		const :ref:`MainProcessLauncherManager::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>`& processLauncherManager
	)

Initialize the simulation.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- engineLauncherManager

		- Engine launchers

	*
		- processLaunchers

		- Process launchers

	*
		- simLock

		- Simulation lock

	*
		- Throws

		- an exception when the initialization fails

.. index:: pair: function; resetSimulation
.. _doxid-class_simulation_manager_1a4d733b33b59c886ac13f572ee25e82e1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool resetSimulation()

Reset the currently running simulation.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- lock

		- Simulation lock

.. index:: pair: function; runSimulationUntilTimeout
.. _doxid-class_simulation_manager_1a844a575a84cc9e3aa55b632cb12f3851:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool runSimulationUntilTimeout(int frac = 1)

Runs the simulation until a separate thread stops it or simTimeout (defined in SimulationConfig) is reached. If simTimeout is zero or negative, ignore it.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- simLock

		- Pass simulation lock if already owned



.. rubric:: Returns:

Returns true if no error was encountered, false otherwise

.. index:: pair: function; runSimulation
.. _doxid-class_simulation_manager_1a561a1f5a5f0bee6f4a129bf3c6f0cc7a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void runSimulation(unsigned numIterations)

Run the Simulation for specified amount of time.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- secs

		- Time (in seconds) to run simulation



.. rubric:: Returns:

Returns true if no error was encountered, false otherwise

.. index:: pair: function; runSimulationOnce
.. _doxid-class_simulation_manager_1a05d7628dfcf057f42bd4f0d041f5a116:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void runSimulationOnce()

Run the Simulation for one timestep.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- secs

		- Time (in seconds) to run simulation



.. rubric:: Returns:

Returns true if no error was encountered, false otherwise

.. index:: pair: function; shutdownLoop
.. _doxid-class_simulation_manager_1a82bf87514e9c01fe615912b427e971bd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void shutdownLoop()

Shuts down simulation loop. Will shutdown any running engines and transceiver functions after any currently running steps are completed.

.. index:: pair: function; configFromParams
.. _doxid-class_simulation_manager_1aec148802afb88105e09b2a7a474ab944:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static :ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>` configFromParams(const cxxopts::ParseResult& args)

Get the config from start parameters.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Parsed start parameters



.. rubric:: Returns:

Returns instance of simulation config

.. index:: pair: function; createFromConfig
.. _doxid-class_simulation_manager_1abc5fe2e0b3b3bd0718d3281637d09320:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static SimulationManager createFromConfig(:ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`& config)

Create :ref:`SimulationManager <doxid-class_simulation_manager>` from pointer to config.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- config

		- Pointer to a config



.. rubric:: Returns:

Returns instance of :ref:`SimulationManager <doxid-class_simulation_manager>`

