.. index:: pair: class; NestJSONServer
.. _doxid-class_nest_j_s_o_n_server:

class NestJSONServer
====================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nest_json_server.h>
	
	class NestJSONServer: public :ref:`EngineJSONServer<doxid-class_engine_j_s_o_n_server>` {
	public:
		// construction
	
		:target:`NestJSONServer<doxid-class_nest_j_s_o_n_server_1af06bfd066d4ea7ed3cb52bbb3ed468df>`(const std::string& serverAddress, boost::python::dict globals);
	
		:target:`NestJSONServer<doxid-class_nest_j_s_o_n_server_1a93ffd6b7376efeec303157d291f9ec31>`(
			const std::string& serverAddress,
			const std::string& engineName,
			const std::string& registrationAddress,
			boost::python::dict globals
		);

		// methods
	
		bool :ref:`initRunFlag<doxid-class_nest_j_s_o_n_server_1a3ebffd2a66218c9fa47ae45be1cc271f>`() const;
		bool :ref:`shutdownFlag<doxid-class_nest_j_s_o_n_server_1ad3b03139b8a747a04571295806b370b4>`() const;
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`runLoopStep<doxid-class_nest_j_s_o_n_server_1a15e32b7efb110783e7774693fc7040d9>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep);
	
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`initialize<doxid-class_nest_j_s_o_n_server_1abd7322673b4ad608186e4e229805b505>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data,
			:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock
		);
	
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`reset<doxid-class_nest_j_s_o_n_server_1a0c06299b3aa7e73fd6c9fbf39ea884d7>`(:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock);
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`shutdown<doxid-class_nest_j_s_o_n_server_1aed7cde182896ad27a877583153ada003>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// typedefs
	
		typedef std::timed_mutex :ref:`mutex_t<doxid-class_engine_j_s_o_n_server_1a5df75e9fa8a25592e4e3ad7064362673>`;
		typedef std::unique_lock<:ref:`EngineJSONServer::mutex_t<doxid-class_engine_j_s_o_n_server_1a5df75e9fa8a25592e4e3ad7064362673>`> :ref:`lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`;

		// methods
	
		:ref:`EngineJSONServer<doxid-class_engine_j_s_o_n_server>`& :ref:`operator =<doxid-class_engine_j_s_o_n_server_1acfacf46dfa94ab1b35047f8ef4ff0d64>` (const :ref:`EngineJSONServer<doxid-class_engine_j_s_o_n_server>`&);
		bool :ref:`isServerRunning<doxid-class_engine_j_s_o_n_server_1a0f65133b9a3a09f6bd2141d135d21e7c>`() const;
		void :ref:`startServerAsync<doxid-class_engine_j_s_o_n_server_1a33acfbda554050d3d47cab00e3d4869e>`();
		void :ref:`startServer<doxid-class_engine_j_s_o_n_server_1a10dbdbcbf4ee934d1b7740c4e3bb11d7>`();
		void :ref:`shutdownServer<doxid-class_engine_j_s_o_n_server_1ad3a67a6b3fb7889725d8474b8631eb10>`();
		uint16_t :ref:`serverPort<doxid-class_engine_j_s_o_n_server_1add529a3d33c10daff532d50790f206e6>`() const;
		std::string :ref:`serverAddress<doxid-class_engine_j_s_o_n_server_1ab98b796faed11c13f43300fc8fdbacb2>`() const;
	
		void :ref:`registerDataPack<doxid-class_engine_j_s_o_n_server_1a20aa263c3e4605edc9b461b270bd6b3b>`(
			const std::string& datapackName,
			:ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`* interface
		);
	
		void :ref:`registerDataPackNoLock<doxid-class_engine_j_s_o_n_server_1a64df4ef44c6a98b8d125cd77bd43bcb6>`(
			const std::string& datapackName,
			:ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`* interface
		);
	
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`runLoopStep<doxid-class_engine_j_s_o_n_server_1a0345dec840e4827786442050a6589787>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep) = 0;
	
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`initialize<doxid-class_engine_j_s_o_n_server_1ac8df612106fa7d2d92663f8fd6cfcfe5>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data,
			:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock
		) = 0;
	
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`reset<doxid-class_engine_j_s_o_n_server_1a76b6062e7a83a6b5adefb01c07547de0>`(:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock) = 0;
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`shutdown<doxid-class_engine_j_s_o_n_server_1a83f8f54978e715ce341af674e1813f90>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data) = 0;

.. _details-class_nest_j_s_o_n_server:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; initRunFlag
.. _doxid-class_nest_j_s_o_n_server_1a3ebffd2a66218c9fa47ae45be1cc271f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool initRunFlag() const

Has the initialization been executed?



.. rubric:: Returns:

Returns true once the initialize function has been run once

.. index:: pair: function; shutdownFlag
.. _doxid-class_nest_j_s_o_n_server_1ad3b03139b8a747a04571295806b370b4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool shutdownFlag() const

Has a shutdown command been received?



.. rubric:: Returns:

Returns true if a shutdown command has been received

.. index:: pair: function; runLoopStep
.. _doxid-class_nest_j_s_o_n_server_1a15e32b7efb110783e7774693fc7040d9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runLoopStep(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep)

Run a single loop step.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeStep

		- Step to take



.. rubric:: Returns:

Returns the time registered by this engine at the end of the loop

.. index:: pair: function; initialize
.. _doxid-class_nest_j_s_o_n_server_1abd7322673b4ad608186e4e229805b505:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` initialize(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data,
		:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock
	)

Engine Initialization routine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Initialization data

	*
		- datapackLock

		- :ref:`DataPack <doxid-class_data_pack>` Lock. Prevents access to _datapacksControllers



.. rubric:: Returns:

Returns data about initialization status

.. index:: pair: function; reset
.. _doxid-class_nest_j_s_o_n_server_1a0c06299b3aa7e73fd6c9fbf39ea884d7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` reset(:ref:`EngineJSONServer::lock_t<doxid-class_engine_j_s_o_n_server_1aa010b9dfa5920648e0605e93213c0c1e>`& datapackLock)

Engine reset routine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackLock

		- :ref:`DataPack <doxid-class_data_pack>` Lock. Prevents access to _datapacksControllers



.. rubric:: Returns:

Returns data about initialization status

.. index:: pair: function; shutdown
.. _doxid-class_nest_j_s_o_n_server_1aed7cde182896ad27a877583153ada003:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` shutdown(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data)

Engine Shutdown routine.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Shutdown data



.. rubric:: Returns:

Returns data about shutdown status

