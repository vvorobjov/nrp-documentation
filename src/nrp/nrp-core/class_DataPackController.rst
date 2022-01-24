.. index:: pair: class; DataPackController
.. _doxid-class_data_pack_controller:

template class DataPackController
=================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Helper class to handle DataPacks on the Engine Server side. :ref:`More...<details-class_data_pack_controller>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <datapack_controller.h>
	
	template <class DATA_TYPE>
	class DataPackController {
	public:
		// methods
	
		virtual DATA_TYPE* :ref:`getDataPackInformation<doxid-class_data_pack_controller_1a357fffcf131e7a69a79a209b25d9c1b7>`() = 0;
		virtual void :ref:`handleDataPackData<doxid-class_data_pack_controller_1ad13d0bef2cbad271cd59a1061ed416f9>`(const DATA_TYPE& data) = 0;
	};

	// direct descendants

	class :ref:`CameraGrpcDataPackController<doxid-classgazebo_1_1_camera_grpc_data_pack_controller>`;
	class :ref:`JointGrpcDataPackController<doxid-classgazebo_1_1_joint_grpc_data_pack_controller>`;
	class :ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`;
	class :ref:`LinkGrpcDataPackController<doxid-classgazebo_1_1_link_grpc_data_pack_controller>`;
.. _details-class_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Helper class to handle DataPacks on the Engine Server side.

As a general policy DataDataPackController is not supposed to take ownership of DATA_TYPE objects which receives or returns. This must be consider for each DataDataPackController specialization.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- DATA_TYPE

		- Object type that this controller handles

Methods
-------

.. index:: pair: function; getDataPackInformation
.. _doxid-class_data_pack_controller_1a357fffcf131e7a69a79a209b25d9c1b7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual DATA_TYPE* getDataPackInformation() = 0

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

.. index:: pair: function; handleDataPackData
.. _doxid-class_data_pack_controller_1ad13d0bef2cbad271cd59a1061ed416f9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void handleDataPackData(const DATA_TYPE& data) = 0

Handle received datapack data.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Data to be processed

