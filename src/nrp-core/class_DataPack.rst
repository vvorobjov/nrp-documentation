.. index:: pair: class; DataPack
.. _doxid-class_data_pack:

template class DataPack
=======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Base datapack class. :ref:`More...<details-class_data_pack>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <datapack.h>
	
	template <class DATA_TYPE>
	class DataPack: public :ref:`DataPackInterface<doxid-class_data_pack_interface>` {
	public:
		// construction
	
		:target:`DataPack<doxid-class_data_pack_1a8db7df8746a95c42b64f34047954a30b>`(
			const std::string& name,
			const std::string& engineName,
			DATA_TYPE* data_
		);
	
		:target:`DataPack<doxid-class_data_pack_1ade9d774a369407f6ce378b03a02a3ebd>`(const std::string& name, const std::string& engineName);
		:target:`DataPack<doxid-class_data_pack_1aa735adbdccf27f106bb4823a8155f16c>`(const DataPack&);

		// methods
	
		DataPack& :target:`operator =<doxid-class_data_pack_1a259fc093eb972e3b43a1fd0ddf1bb7fd>` (const DataPack&);
		const DATA_TYPE& :ref:`getData<doxid-class_data_pack_1a3ca8d37fbc23195de656d24592b30834>`() const;
		DATA_TYPE* :ref:`releaseData<doxid-class_data_pack_1a5366c141262fa29297ad0997556ecd85>`();
		PyObject* :ref:`toPythonString<doxid-class_data_pack_1a6083d8d85478dc68e2ccfd6d8091e63f>`();
		virtual :ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>` :target:`moveToSharedPtr<doxid-class_data_pack_1ad33b943ed2d151c54eef0b9da0d4ee5b>`();
		virtual :ref:`DataPackInterface<doxid-class_data_pack_interface>`* :ref:`clone<doxid-class_data_pack_1a6480f49617693f976a876b80e2671fdd>`() const;
		static std::string :ref:`getType<doxid-class_data_pack_1ae6b41eeae3e3616ec74051b493b50c86>`();
	
		static :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>` :ref:`createID<doxid-class_data_pack_1a6d6ec1e24e2e6bedd43a631e1ed49e3b>`(
			const std::string& name,
			const std::string& engineName
		);
	
		static void :target:`create_python<doxid-class_data_pack_1af9fe16a5a96e0a7658a0923b2618d056>`(const std::string& name);
	};

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

		// methods
	
		const std::string& :ref:`name<doxid-class_data_pack_interface_1aeb28f4ade551550a6371585dd8014225>`() const;
		void :ref:`setName<doxid-class_data_pack_interface_1ad213cfe70ff5e2e17feed278f46ae01d>`(const std::string& name);
		const std::string& :ref:`type<doxid-class_data_pack_interface_1afa907c903c09daa0ec4ab84af037ef57>`() const;
		void :ref:`setType<doxid-class_data_pack_interface_1aa268ef7e5d1abc7953c39681269bad53>`(const std::string& type);
		const std::string& :ref:`engineName<doxid-class_data_pack_interface_1ade2caf78751f319aabacd46d2633157e>`() const;
		void :ref:`setEngineName<doxid-class_data_pack_interface_1a87b6065c95b44207ee29bf0273ca16b6>`(const std::string& engineName);
		const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& :ref:`id<doxid-class_data_pack_interface_1a6734398f7fd597ed8624c4d4b1c83fc4>`() const;
		void :ref:`setID<doxid-class_data_pack_interface_1ad7e2d46c439a928a426042cdd9f715a8>`(const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& id);
		virtual :ref:`DataPackInterface::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :ref:`moveToSharedPtr<doxid-class_data_pack_interface_1a769e9f29e1a89e7010d0b4bb70f51b83>`();
		virtual :ref:`DataPackInterface<doxid-class_data_pack_interface>`* :ref:`clone<doxid-class_data_pack_interface_1a02a9996b2349806d5c7514ad5130d0d0>`() const;
		bool :ref:`isEmpty<doxid-class_data_pack_interface_1ae29518b89988c7e7639765ff8d91977e>`() const;

.. _details-class_data_pack:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base datapack class.

The class must be specialized by providing a template argument. The argument defines what data class will be stored in the datapack objects.

Methods
-------

.. index:: pair: function; getData
.. _doxid-class_data_pack_1a3ca8d37fbc23195de656d24592b30834:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const DATA_TYPE& getData() const

Returns reference to data stored in the object.

The function returns a read-only reference to the data stored by the object. This is the main accessor function of the :ref:`DataPack <doxid-class_data_pack>` object.



.. rubric:: Returns:

Read-only reference to the data stored by the object

.. index:: pair: function; releaseData
.. _doxid-class_data_pack_1a5366c141262fa29297ad0997556ecd85:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	DATA_TYPE* releaseData()

Releases ownership of the data stored in the object and returns a raw pointer to the data.

The caller is responsible for destruction of the released data.



.. rubric:: Returns:

Raw pointer to the data stored in the object

.. index:: pair: function; toPythonString
.. _doxid-class_data_pack_1a6083d8d85478dc68e2ccfd6d8091e63f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	PyObject* toPythonString()

Returns a python string representation of this object content.



.. rubric:: Returns:

Python string

.. index:: pair: function; clone
.. _doxid-class_data_pack_1a6480f49617693f976a876b80e2671fdd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`DataPackInterface<doxid-class_data_pack_interface>`* clone() const

Virtual clone method to support polymorphic copy.

.. index:: pair: function; getType
.. _doxid-class_data_pack_1ae6b41eeae3e3616ec74051b493b50c86:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static std::string getType()

Returns type of the datapack class.

The function returns type of the datapack class as string. The return value is not human readable. It's an implementation-defined name of the DATA_TYPE used by the datapack.

.. index:: pair: function; createID
.. _doxid-class_data_pack_1a6d6ec1e24e2e6bedd43a631e1ed49e3b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>` createID(
		const std::string& name,
		const std::string& engineName
	)

Creates a :ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>` object with type matching the DATA_TYPE used by the :ref:`DataPack <doxid-class_data_pack>` class.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- name

		- Name of the datapack

	*
		- name

		- Name of the engine to which the datapack belongs



.. rubric:: Returns:

A :ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>` object, with :ref:`DataPackIdentifier::Name <doxid-struct_data_pack_identifier_1a4503921eb790287b4934104fe19d870b>` and :ref:`DataPackIdentifier::EngineName <doxid-struct_data_pack_identifier_1a0f52d05427bba45a3bc49a6aa690d2f7>` provided as arguments, and with :ref:`DataPackIdentifier::Type <doxid-struct_data_pack_identifier_1a39e482341dca27cee33a6d7d78f99605>` deduced from DATA_TYPE template argument of the :ref:`DataPack <doxid-class_data_pack>` class.

