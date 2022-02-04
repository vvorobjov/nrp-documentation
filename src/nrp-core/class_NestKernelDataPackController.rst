.. index:: pair: class; NestKernelDataPackController
.. _doxid-class_nest_kernel_data_pack_controller:

class NestKernelDataPackController
==================================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nest_kernel_datapack_controller.h>
	
	class NestKernelDataPackController: public :ref:`JsonDataPackController<doxid-class_json_data_pack_controller>` {
	public:
		// construction
	
		:target:`NestKernelDataPackController<doxid-class_nest_kernel_data_pack_controller_1af2a9e70166cda19e30f34c40d84c3a06>`(
			const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& devID,
			boost::python::dict nest
		);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-class_nest_kernel_data_pack_controller_1afb54be44b71bea82530eb6a0b386ed8d>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* :ref:`getDataPackInformation<doxid-class_nest_kernel_data_pack_controller_1a0fcc7395f404d4657856c06a7f7ac0f9>`();
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

.. _details-class_nest_kernel_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-class_nest_kernel_data_pack_controller_1afb54be44b71bea82530eb6a0b386ed8d:

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
.. _doxid-class_nest_kernel_data_pack_controller_1a0fcc7395f404d4657856c06a7f7ac0f9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

