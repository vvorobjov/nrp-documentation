.. index:: pair: class; gazebo::JointGrpcDataPackController
.. _doxid-classgazebo_1_1_joint_grpc_data_pack_controller:

class gazebo::JointGrpcDataPackController
=========================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Interface for a single joint. :ref:`More...<details-classgazebo_1_1_joint_grpc_data_pack_controller>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <joint_datapack_controller.h>
	
	class JointGrpcDataPackController: public :ref:`DataPackController<doxid-class_data_pack_controller>` {
	public:
		// construction
	
		:target:`JointGrpcDataPackController<doxid-classgazebo_1_1_joint_grpc_data_pack_controller_1a99560983c5041601fd990400b2c9432a>`(
			const std::string& jointName,
			const physics::JointPtr& joint,
			const physics::JointControllerPtr& jointController
		);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-classgazebo_1_1_joint_grpc_data_pack_controller_1a385db35cfefdf3eafa31a29ac8cf2ada>`(const google::protobuf::Message& data);
		virtual google::protobuf::Message* :ref:`getDataPackInformation<doxid-classgazebo_1_1_joint_grpc_data_pack_controller_1affdfd021168f3cab6e7f8cc840c5363b>`();
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		virtual DATA_TYPE* :ref:`getDataPackInformation<doxid-class_data_pack_controller_1a357fffcf131e7a69a79a209b25d9c1b7>`() = 0;
		virtual void :ref:`handleDataPackData<doxid-class_data_pack_controller_1ad13d0bef2cbad271cd59a1061ed416f9>`(const DATA_TYPE& data) = 0;

.. _details-classgazebo_1_1_joint_grpc_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Interface for a single joint.

Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-classgazebo_1_1_joint_grpc_data_pack_controller_1a385db35cfefdf3eafa31a29ac8cf2ada:

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
.. _doxid-classgazebo_1_1_joint_grpc_data_pack_controller_1affdfd021168f3cab6e7f8cc840c5363b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual google::protobuf::Message* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

