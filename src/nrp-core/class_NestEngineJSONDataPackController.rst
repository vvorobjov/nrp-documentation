.. index:: pair: class; NestEngineJSONDataPackController
.. _doxid-class_nest_engine_j_s_o_n_data_pack_controller:

class NestEngineJSONDataPackController
======================================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nest_engine_datapack_controller.h>
	
	class NestEngineJSONDataPackController: public :ref:`JsonDataPackController<doxid-class_json_data_pack_controller>` {
	public:
		// construction
	
		:target:`NestEngineJSONDataPackController<doxid-class_nest_engine_j_s_o_n_data_pack_controller_1a53bae27202386957a829ff924c512126>`(
			const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& devID,
			boost::python::object nodeCollection,
			boost::python::dict nest
		);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-class_nest_engine_j_s_o_n_data_pack_controller_1a40fdf27e12d1a056ebee40b23942de36>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* :ref:`getDataPackInformation<doxid-class_nest_engine_j_s_o_n_data_pack_controller_1a6d04dddc266f8b0509194e8b4c0f7936>`();
		void :ref:`setNestID<doxid-class_nest_engine_j_s_o_n_data_pack_controller_1ad2e18b21813539ff1e6b819f131d8db5>`(boost::python::dict nest, boost::python::object nodeCollection);
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

.. _details-class_nest_engine_j_s_o_n_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-class_nest_engine_j_s_o_n_data_pack_controller_1a40fdf27e12d1a056ebee40b23942de36:

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
.. _doxid-class_nest_engine_j_s_o_n_data_pack_controller_1a6d04dddc266f8b0509194e8b4c0f7936:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

.. index:: pair: function; setNestID
.. _doxid-class_nest_engine_j_s_o_n_data_pack_controller_1ad2e18b21813539ff1e6b819f131d8db5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void setNestID(boost::python::dict nest, boost::python::object nodeCollection)

Set Nest properties.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- nest

		- Nest instance

	*
		- nodeCollection

		- Nest GIDs of model managed by this controller

