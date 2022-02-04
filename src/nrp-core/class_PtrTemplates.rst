.. index:: pair: class; PtrTemplates
.. _doxid-class_ptr_templates:

template class PtrTemplates
===========================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <ptr_templates.h>
	
	template <class T>
	class PtrTemplates {
	public:
		// typedefs
	
		typedef std::shared_ptr<T> :target:`shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`;
		typedef std::shared_ptr<const T> :target:`const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>`;
		typedef std::unique_ptr<T> :target:`unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`;
		typedef std::unique_ptr<const T> :target:`const_unique_ptr<doxid-class_ptr_templates_1aef0eb44f9c386dbf0de54d0f5afac667>`;
	};

	// direct descendants

	class :ref:`DataPackInterface<doxid-class_data_pack_interface>`;
	class :ref:`EngineClientInterface<doxid-class_engine_client_interface>`;
	class :ref:`EngineLauncherInterface<doxid-class_engine_launcher_interface>`;
	class :ref:`EngineLauncherManager<doxid-class_engine_launcher_manager>`;
	class :ref:`FTILoop<doxid-class_f_t_i_loop>`;
	class :ref:`LaunchCommandInterface<doxid-class_launch_command_interface>`;
	class :ref:`ProcessLauncherInterface<doxid-class_process_launcher_interface>`;

	template <class ... PROCESS_LAUNCHERS>
	class :ref:`ProcessLauncherManager<doxid-class_process_launcher_manager>`;

	class :ref:`PyEngineScript<doxid-class_py_engine_script>`;
	class :ref:`SimulationManager<doxid-class_simulation_manager>`;
	class :ref:`TransceiverDataPackInterface<doxid-class_transceiver_data_pack_interface>`;
	class :ref:`TransceiverFunction<doxid-class_transceiver_function>`;
