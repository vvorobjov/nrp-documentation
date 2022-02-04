.. index:: pair: class; DataPackInterface
.. _doxid-class_data_pack_interface:

class DataPackInterface
=======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Interface to datapacks. :ref:`More...<details-class_data_pack_interface>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <datapack_interface.h>
	
	class DataPackInterface: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// construction
	
		:target:`DataPackInterface<doxid-class_data_pack_interface_1a7e7e5afe30c32e7ab1080aa9470c1554>`();
	
		template <class DEV_ID_T>
		:target:`DataPackInterface<doxid-class_data_pack_interface_1a30491f12e0073686ccd2f719dc96ca30>`(DEV_ID_T&& id);
	
		:target:`DataPackInterface<doxid-class_data_pack_interface_1a6c84cdf091ba49a21cfa2658fc13fd93>`(
			const std::string& name,
			const std::string& engineName,
			const std::string& type
		);

		// methods
	
		const std::string& :target:`name<doxid-class_data_pack_interface_1aeb28f4ade551550a6371585dd8014225>`() const;
		void :target:`setName<doxid-class_data_pack_interface_1ad213cfe70ff5e2e17feed278f46ae01d>`(const std::string& name);
		const std::string& :target:`type<doxid-class_data_pack_interface_1afa907c903c09daa0ec4ab84af037ef57>`() const;
		void :target:`setType<doxid-class_data_pack_interface_1aa268ef7e5d1abc7953c39681269bad53>`(const std::string& type);
		const std::string& :target:`engineName<doxid-class_data_pack_interface_1ade2caf78751f319aabacd46d2633157e>`() const;
		void :target:`setEngineName<doxid-class_data_pack_interface_1a87b6065c95b44207ee29bf0273ca16b6>`(const std::string& engineName);
		const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& :target:`id<doxid-class_data_pack_interface_1a6734398f7fd597ed8624c4d4b1c83fc4>`() const;
		void :target:`setID<doxid-class_data_pack_interface_1ad7e2d46c439a928a426042cdd9f715a8>`(const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& id);
		virtual :ref:`DataPackInterface::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`moveToSharedPtr<doxid-class_data_pack_interface_1a769e9f29e1a89e7010d0b4bb70f51b83>`();
		virtual DataPackInterface* :ref:`clone<doxid-class_data_pack_interface_1a02a9996b2349806d5c7514ad5130d0d0>`() const;
		bool :ref:`isEmpty<doxid-class_data_pack_interface_1ae29518b89988c7e7639765ff8d91977e>`() const;
	};

	// direct descendants

	template <class DATA_TYPE>
	class :ref:`DataPack<doxid-class_data_pack>`;

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// typedefs
	
		typedef std::shared_ptr<T> :ref:`shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`;
		typedef std::shared_ptr<const T> :ref:`const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>`;
		typedef std::unique_ptr<T> :ref:`unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`;
		typedef std::unique_ptr<const T> :ref:`const_unique_ptr<doxid-class_ptr_templates_1aef0eb44f9c386dbf0de54d0f5afac667>`;

.. _details-class_data_pack_interface:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Interface to datapacks.

Methods
-------

.. index:: pair: function; clone
.. _doxid-class_data_pack_interface_1a02a9996b2349806d5c7514ad5130d0d0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual DataPackInterface* clone() const

Virtual clone method to support polymorphic copy.

.. index:: pair: function; isEmpty
.. _doxid-class_data_pack_interface_1ae29518b89988c7e7639765ff8d91977e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool isEmpty() const

Indicates if the datapack contains any data aside from datapack ID.

The function will return true, if the datapack is of :ref:`DataPackInterface <doxid-class_data_pack_interface>` type, which contains only datapack ID. For any concrete implementation of :ref:`DataPack <doxid-class_data_pack>` class, it should return false.

