.. index:: pair: class; gazebo::LinkGrpcDataPackController
.. _doxid-classgazebo_1_1_link_grpc_data_pack_controller:

class gazebo::LinkGrpcDataPackController
========================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Interface for links. :ref:`More...<details-classgazebo_1_1_link_grpc_data_pack_controller>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <link_datapack_controller.h>
	
	class LinkGrpcDataPackController: public :ref:`DataPackController<doxid-class_data_pack_controller>` {
	public:
		// construction
	
		:target:`LinkGrpcDataPackController<doxid-classgazebo_1_1_link_grpc_data_pack_controller_1a036196f094abb4090bec45a10ae24836>`(
			const std::string& linkName,
			const physics::LinkPtr& link
		);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-classgazebo_1_1_link_grpc_data_pack_controller_1af75208f327c212c65f3f59001eb7c633>`(const google::protobuf::Message& data);
		virtual google::protobuf::Message* :ref:`getDataPackInformation<doxid-classgazebo_1_1_link_grpc_data_pack_controller_1a7d49026c1804ee08e67e78076b58ba5c>`();
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		virtual DATA_TYPE* :ref:`getDataPackInformation<doxid-class_data_pack_controller_1a357fffcf131e7a69a79a209b25d9c1b7>`() = 0;
		virtual void :ref:`handleDataPackData<doxid-class_data_pack_controller_1ad13d0bef2cbad271cd59a1061ed416f9>`(const DATA_TYPE& data) = 0;

.. _details-classgazebo_1_1_link_grpc_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Interface for links.

Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-classgazebo_1_1_link_grpc_data_pack_controller_1af75208f327c212c65f3f59001eb7c633:

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
.. _doxid-classgazebo_1_1_link_grpc_data_pack_controller_1a7d49026c1804ee08e67e78076b58ba5c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual google::protobuf::Message* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

