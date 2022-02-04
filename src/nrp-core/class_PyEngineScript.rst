.. index:: pair: class; PyEngineScript
.. _doxid-class_py_engine_script:

class PyEngineScript
====================

.. toctree::
	:hidden:

Overview
~~~~~~~~

C++ class to interface with user-defined Python script class. :ref:`More...<details-class_py_engine_script>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <py_engine_script.h>
	
	class PyEngineScript: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// methods
	
		virtual void :ref:`initialize<doxid-class_py_engine_script_1a92d8c188507593344c005f23dbb94210>`();
		:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`runLoop<doxid-class_py_engine_script_1af437695ced79628f5c6f290796faf424>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timestep);
		virtual void :ref:`shutdown<doxid-class_py_engine_script_1a38abcb6efff825e89a92c31296dcb2fc>`();
		virtual bool :ref:`reset<doxid-class_py_engine_script_1aaf5bb6e36b3ccfd938d12d84fdb1f8f0>`();
		:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`simTime<doxid-class_py_engine_script_1a3fda00c81e45ed7a5e7269ceca3a5577>`() const;
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`engineConfig<doxid-class_py_engine_script_1a3df28377fac9f228d7a6198a05036ec6>`() const;
		void :ref:`registerDataPack<doxid-class_py_engine_script_1ab8fb2d485c32900370537aa73d9868f5>`(std::string datapackName);
		boost::python::object& :ref:`getDataPack<doxid-class_py_engine_script_1a801feb6fc18980d8dd2b4e5f99df34fa>`(const std::string& datapackName);
		void :ref:`setDataPack<doxid-class_py_engine_script_1a19bf2fdec4c1148d1dbe056363e54bc0>`(const std::string& datapackName, boost::python::object data);
		void :ref:`setPythonJSONServer<doxid-class_py_engine_script_1aa9b01d510a780f4bdbba873a634a54e6>`(:ref:`PythonJSONServer<doxid-class_python_j_s_o_n_server>`* pServer);
	};

	// direct descendants

	struct :ref:`PyEngineScriptWrapper<doxid-struct_py_engine_script_wrapper>`;

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

.. _details-class_py_engine_script:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

C++ class to interface with user-defined Python script class.

Methods
-------

.. index:: pair: function; initialize
.. _doxid-class_py_engine_script_1a92d8c188507593344c005f23dbb94210:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void initialize()

Initialization function. Called at start of simulation.

.. index:: pair: function; runLoop
.. _doxid-class_py_engine_script_1af437695ced79628f5c6f290796faf424:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runLoop(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timestep)

Runs main script loop and updates _time.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timestep

		- Time (in seconds) to run the loop



.. rubric:: Returns:

Returns total simulation time of this engine

.. index:: pair: function; shutdown
.. _doxid-class_py_engine_script_1a38abcb6efff825e89a92c31296dcb2fc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void shutdown()

Shutdown function. Called at end of simulation.

.. index:: pair: function; reset
.. _doxid-class_py_engine_script_1aaf5bb6e36b3ccfd938d12d84fdb1f8f0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual bool reset()

Reset function.

.. index:: pair: function; simTime
.. _doxid-class_py_engine_script_1a3fda00c81e45ed7a5e7269ceca3a5577:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` simTime() const

Get simulation time of this engine.

.. index:: pair: function; engineConfig
.. _doxid-class_py_engine_script_1a3df28377fac9f228d7a6198a05036ec6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` engineConfig() const

Get this engine configuration.

.. index:: pair: function; registerDataPack
.. _doxid-class_py_engine_script_1ab8fb2d485c32900370537aa73d9868f5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerDataPack(std::string datapackName)

Register datapack.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackName

		- Name of datapack

.. index:: pair: function; getDataPack
.. _doxid-class_py_engine_script_1a801feb6fc18980d8dd2b4e5f99df34fa:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object& getDataPack(const std::string& datapackName)

Get :ref:`DataPack <doxid-class_data_pack>` Data.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackName

		- Name of datapack



.. rubric:: Returns:

Returns datapack data

.. index:: pair: function; setDataPack
.. _doxid-class_py_engine_script_1a19bf2fdec4c1148d1dbe056363e54bc0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void setDataPack(const std::string& datapackName, boost::python::object data)

Set :ref:`DataPack <doxid-class_data_pack>` Data.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackName

		- Name of datapack

	*
		- data

		- Data to store in datapack

.. index:: pair: function; setPythonJSONServer
.. _doxid-class_py_engine_script_1aa9b01d510a780f4bdbba873a634a54e6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void setPythonJSONServer(:ref:`PythonJSONServer<doxid-class_python_j_s_o_n_server>`* pServer)

Save ptr to :ref:`PythonJSONServer <doxid-class_python_j_s_o_n_server>` instance that owns this script.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- pServer

		- Pointer to :ref:`PythonJSONServer <doxid-class_python_j_s_o_n_server>`

