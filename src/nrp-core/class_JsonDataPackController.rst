.. index:: pair: class; JsonDataPackController
.. _doxid-class_json_data_pack_controller:

class JsonDataPackController
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Base controller class for JSON datapacks. :ref:`More...<details-class_json_data_pack_controller>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <json_datapack_controller.h>
	
	class JsonDataPackController: public :ref:`DataPackController<doxid-class_data_pack_controller>` {
	public:
		// methods
	
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :target:`getEmptyDataPack<doxid-class_json_data_pack_controller_1a23f08f76aa5fc165adc7020ae67f0dbf>`() const;
	};

	// direct descendants

	class :ref:`CameraDataPackController<doxid-classgazebo_1_1_camera_data_pack_controller>`;
	class :ref:`JointDataPackController<doxid-classgazebo_1_1_joint_data_pack_controller>`;
	class :ref:`LinkDataPackController<doxid-classgazebo_1_1_link_data_pack_controller>`;
	class :ref:`NestEngineJSONDataPackController<doxid-class_nest_engine_j_s_o_n_data_pack_controller>`;
	class :ref:`NestKernelDataPackController<doxid-class_nest_kernel_data_pack_controller>`;
	class :ref:`PythonEngineJSONDataPackController<doxid-class_python_engine_j_s_o_n_data_pack_controller>`;

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		virtual DATA_TYPE* :ref:`getDataPackInformation<doxid-class_data_pack_controller_1a357fffcf131e7a69a79a209b25d9c1b7>`() = 0;
		virtual void :ref:`handleDataPackData<doxid-class_data_pack_controller_1ad13d0bef2cbad271cd59a1061ed416f9>`(const DATA_TYPE& data) = 0;

.. _details-class_json_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base controller class for JSON datapacks.

The class provides helper methods and common members for controllers that are used to handle JSON datapacks.

