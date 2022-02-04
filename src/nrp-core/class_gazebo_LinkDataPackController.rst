.. index:: pair: class; gazebo::LinkDataPackController
.. _doxid-classgazebo_1_1_link_data_pack_controller:

class gazebo::LinkDataPackController
====================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Interface for links. :ref:`More...<details-classgazebo_1_1_link_data_pack_controller>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <link_datapack_controller.h>
	
	class LinkDataPackController: public :ref:`JsonDataPackController<doxid-class_json_data_pack_controller>` {
	public:
		// construction
	
		:target:`LinkDataPackController<doxid-classgazebo_1_1_link_data_pack_controller_1a007c6a5a04358530ec10d62153b51905>`(
			const std::string& linkName,
			const physics::LinkPtr& link
		);

		// methods
	
		virtual void :ref:`handleDataPackData<doxid-classgazebo_1_1_link_data_pack_controller_1abec92a4c34dd72c099fd1a030ec55736>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& data);
		virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* :ref:`getDataPackInformation<doxid-classgazebo_1_1_link_data_pack_controller_1a059ef76c59ef22a3ba77363f263b5021>`();
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

.. _details-classgazebo_1_1_link_data_pack_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Interface for links.

Methods
-------

.. index:: pair: function; handleDataPackData
.. _doxid-classgazebo_1_1_link_data_pack_controller_1abec92a4c34dd72c099fd1a030ec55736:

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
.. _doxid-classgazebo_1_1_link_data_pack_controller_1a059ef76c59ef22a3ba77363f263b5021:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* getDataPackInformation()

Get datapack information to be forwarded to the NRP.



.. rubric:: Returns:

Returns a DATA_TYPE pointer containing requested data

