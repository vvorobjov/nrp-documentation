.. index:: pair: class; gazebo::CameraGrpcDataPackController
.. _doxid-classgazebo_1_1_camera_grpc_data_pack_controller:

class gazebo::CameraGrpcDataPackController
==========================================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <camera_datapack_controller.h>
	
	class CameraGrpcDataPackController: public :ref:`DataPackController<doxid-class_data_pack_controller>` {
	public:
		// construction
	
		:target:`CameraGrpcDataPackController<doxid-classgazebo_1_1_camera_grpc_data_pack_controller_1a3e627d55a1ef298c81287c5a37692994>`(
			const std::string& devName,
			const rendering::CameraPtr& camera,
			const sensors::SensorPtr& parent
		);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-classgazebo_1_1_camera_grpc_data_pack_controller_1a3f5454e29467ce049782aaf30c3b4806>`(const google::protobuf::Message& data);
		virtual google::protobuf::Message* :ref:`getDataPackInformation<doxid-classgazebo_1_1_camera_grpc_data_pack_controller_1ad261522b254b3b451761989b13b4dee6>`();
	
		void :target:`updateCamData<doxid-classgazebo_1_1_camera_grpc_data_pack_controller_1a3a8cea2548a89b5827c8ebeb6c005e08>`(
			const unsigned char* image,
			unsigned int width,
			unsigned int height,
			unsigned int depth
		);
	
		void :target:`resetTime<doxid-classgazebo_1_1_camera_grpc_data_pack_controller_1af22b2c34ab706043b298d0dba5fd8b50>`();
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		virtual DATA_TYPE* :ref:`getDataPackInformation<doxid-class_data_pack_controller_1a357fffcf131e7a69a79a209b25d9c1b7>`() = 0;
		virtual void :ref:`handleDataPackData<doxid-class_data_pack_controller_1ad13d0bef2cbad271cd59a1061ed416f9>`(const DATA_TYPE& data) = 0;

.. _details-classgazebo_1_1_camera_grpc_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-classgazebo_1_1_camera_grpc_data_pack_controller_1a3f5454e29467ce049782aaf30c3b4806:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void handleDataPackData(const google::protobuf::Message& data)

Handle received datapack data.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Data to be processed

.. index:: pair: function; getDataPackInformation
.. _doxid-classgazebo_1_1_camera_grpc_data_pack_controller_1ad261522b254b3b451761989b13b4dee6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual google::protobuf::Message* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

