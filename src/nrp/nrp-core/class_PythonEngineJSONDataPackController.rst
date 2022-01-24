.. index:: pair: class; PythonEngineJSONDataPackController
.. _doxid-class_python_engine_j_s_o_n_data_pack_controller:

class PythonEngineJSONDataPackController
========================================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <python_engine_json_datapack_controller.h>
	
	class PythonEngineJSONDataPackController: public :ref:`JsonDataPackController<doxid-class_json_data_pack_controller>` {
	public:
		// construction
	
		:target:`PythonEngineJSONDataPackController<doxid-class_python_engine_j_s_o_n_data_pack_controller_1a91994bba0f6b058fceefe12b62ad15f0>`(const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& devID);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-class_python_engine_j_s_o_n_data_pack_controller_1a4298f2237c6451eeb9f7e1ea9b259f16>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* :ref:`getDataPackInformation<doxid-class_python_engine_j_s_o_n_data_pack_controller_1a384ec8538dc09cfac9a60a235f42b37c>`();
		boost::python::object& :ref:`data<doxid-class_python_engine_j_s_o_n_data_pack_controller_1a9486d0973034c2d02e1e56ac8a341bcf>`();
		boost::python::object :target:`data<doxid-class_python_engine_j_s_o_n_data_pack_controller_1a6599cac821ce9530a46d636e53e8d66a>`() const;
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

.. _details-class_python_engine_j_s_o_n_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-class_python_engine_j_s_o_n_data_pack_controller_1a4298f2237c6451eeb9f7e1ea9b259f16:

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
.. _doxid-class_python_engine_j_s_o_n_data_pack_controller_1a384ec8538dc09cfac9a60a235f42b37c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

.. index:: pair: function; data
.. _doxid-class_python_engine_j_s_o_n_data_pack_controller_1a9486d0973034c2d02e1e56ac8a341bcf:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object& data()

Get python object referenced by this controller.

