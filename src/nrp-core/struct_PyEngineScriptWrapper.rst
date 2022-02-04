.. index:: pair: struct; PyEngineScriptWrapper
.. _doxid-struct_py_engine_script_wrapper:

struct PyEngineScriptWrapper
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Wrapper around :ref:`PyEngineScript <doxid-class_py_engine_script>`. Used to make derived python classes available from C++. :ref:`More...<details-struct_py_engine_script_wrapper>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <py_engine_script_wrapper.h>
	
	struct PyEngineScriptWrapper:
	    public :ref:`PyEngineScript<doxid-class_py_engine_script>`,
	    public boost::python::wrapper< PyEngineScript > {
		// methods
	
		virtual void :ref:`initialize<doxid-struct_py_engine_script_wrapper_1a11dcb1c3468788ebfd621b56cc5d5724>`();
		void :target:`defaultInitialize<doxid-struct_py_engine_script_wrapper_1aea845cdeebfbc7768d58ffbea52eb1fc>`();
		virtual void :ref:`runLoopFcn<doxid-struct_py_engine_script_wrapper_1afc9e60d1641e6b43ee8180bf0d42dbae>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timestep);
		virtual void :ref:`shutdown<doxid-struct_py_engine_script_wrapper_1a74e9cce086c54c29d9a01f438839299f>`();
		void :target:`defaultShutdown<doxid-struct_py_engine_script_wrapper_1ab9f22a40ffc4b2c00df03265a03271b4>`();
		virtual bool :ref:`reset<doxid-struct_py_engine_script_wrapper_1ae1b136602fb10478c35eaeac4201a634>`();
		bool :target:`defaultReset<doxid-struct_py_engine_script_wrapper_1afeac13b7a3a48b29d90726c09433b21c>`();
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

.. _details-struct_py_engine_script_wrapper:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Wrapper around :ref:`PyEngineScript <doxid-class_py_engine_script>`. Used to make derived python classes available from C++.

Methods
-------

.. index:: pair: function; initialize
.. _doxid-struct_py_engine_script_wrapper_1a11dcb1c3468788ebfd621b56cc5d5724:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void initialize()

Initialization function. Called at start of simulation.

.. index:: pair: function; runLoopFcn
.. _doxid-struct_py_engine_script_wrapper_1afc9e60d1641e6b43ee8180bf0d42dbae:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void runLoopFcn(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timestep)

Main script loop. Will run for timestep seconds.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timestep

		- Time (in seconds) to run the loop

.. index:: pair: function; shutdown
.. _doxid-struct_py_engine_script_wrapper_1a74e9cce086c54c29d9a01f438839299f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void shutdown()

Shutdown function. Called at end of simulation.

.. index:: pair: function; reset
.. _doxid-struct_py_engine_script_wrapper_1ae1b136602fb10478c35eaeac4201a634:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual bool reset()

Reset function.

