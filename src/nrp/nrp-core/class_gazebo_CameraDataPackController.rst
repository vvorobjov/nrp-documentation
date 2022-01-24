.. index:: pair: class; gazebo::CameraDataPackController
.. _doxid-classgazebo_1_1_camera_data_pack_controller:

class gazebo::CameraDataPackController
======================================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <camera_datapack_controller.h>
	
	class CameraDataPackController: public :ref:`JsonDataPackController<doxid-class_json_data_pack_controller>` {
	public:
		// construction
	
		:target:`CameraDataPackController<doxid-classgazebo_1_1_camera_data_pack_controller_1a947aa0ca6bee6b5a8b02fd0554c950f0>`(
			const std::string& devName,
			const rendering::CameraPtr& camera,
			const sensors::SensorPtr& parent
		);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-classgazebo_1_1_camera_data_pack_controller_1a6b8b24afc43f2cecd9ebeaa56bd6032c>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* :ref:`getDataPackInformation<doxid-classgazebo_1_1_camera_data_pack_controller_1a6386a48c86c3ff24f198f1d9f4cc782c>`();
	
		void :target:`updateCamData<doxid-classgazebo_1_1_camera_data_pack_controller_1a7ca654ed06fa03e63d89677bdbb3849a>`(
			const unsigned char* image,
			unsigned int width,
			unsigned int height,
			unsigned int depth
		);
	
		void :target:`resetTime<doxid-classgazebo_1_1_camera_data_pack_controller_1a4ef9f9b9a57b1ec99bc0cde193c92d7a>`();
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		virtual DATA_TYPE* :ref:`getDataPackInformation<doxid-class_data_pack_controller_1a357fffcf131e7a69a79a209b25d9c1b7>`() = 0;
		virtual void :ref:`handleDataPackData<doxid-class_data_pack_controller_1ad13d0bef2cbad271cd59a1061ed416f9>`(const DATA_TYPE& data) = 0;
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& :ref:`getEmptyDataPack<doxid-class_json_data_pack_controller_1a23f08f76aa5fc165adc7020ae67f0dbf>`() const;

.. _details-classgazebo_1_1_camera_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-classgazebo_1_1_camera_data_pack_controller_1a6b8b24afc43f2cecd9ebeaa56bd6032c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void handleDataPackData(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data)

Handle received datapack data.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Data to be processed

.. index:: pair: function; getDataPackInformation
.. _doxid-classgazebo_1_1_camera_data_pack_controller_1a6386a48c86c3ff24f198f1d9f4cc782c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

