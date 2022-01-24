.. index:: pair: class; TransceiverDataPackInterface
.. _doxid-class_transceiver_data_pack_interface:

class TransceiverDataPackInterface
==================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Base of TF Decorators. :ref:`More...<details-class_transceiver_data_pack_interface>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <transceiver_datapack_interface.h>
	
	class TransceiverDataPackInterface: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// methods
	
		template <class TRANSCEIVER_DATAPACK>
		:ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :ref:`pySetup<doxid-class_transceiver_data_pack_interface_1ad14f7dc9eb8b8e09e4281fbf20a9583e>`(const :ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`& tfDataPack);
	
		virtual const std::string& :ref:`linkedEngineName<doxid-class_transceiver_data_pack_interface_1a0857c957956587af7a92f981a26ec987>`() const;
		virtual bool :ref:`isPrepocessing<doxid-class_transceiver_data_pack_interface_1aa10d5676bd9cbcfd93afddda59e077fd>`() const;
	
		virtual boost::python::object :ref:`runTf<doxid-class_transceiver_data_pack_interface_1aebff4b55ef032d519cf1ee594ba76806>`(
			boost::python::tuple& args,
			boost::python::dict& kwargs
		);
	
		virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :ref:`updateRequestedDataPackIDs<doxid-class_transceiver_data_pack_interface_1acb6252267d56aa7520bb5591ed69793d>`(:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`&& datapackIDs = :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`()) const;
		virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :ref:`getRequestedDataPackIDs<doxid-class_transceiver_data_pack_interface_1aa2721b1c08b446bb21682f4c37e5cea0>`() const;
		static void :ref:`setTFInterpreter<doxid-class_transceiver_data_pack_interface_1a271c6f176f41d1110e3316fe4653941a>`(:ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`* interpreter);
	};

	// direct descendants

	class :ref:`EngineDataPack<doxid-class_engine_data_pack>`;
	struct :ref:`TransceiverDataPackInterfaceWrapper<doxid-struct_transceiver_data_pack_interface_wrapper>`;
	class :ref:`TransceiverFunction<doxid-class_transceiver_function>`;

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

.. _details-class_transceiver_data_pack_interface:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base of TF Decorators.

Methods
-------

.. index:: pair: function; pySetup
.. _doxid-class_transceiver_data_pack_interface_1ad14f7dc9eb8b8e09e4281fbf20a9583e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class TRANSCEIVER_DATAPACK>
	:ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` pySetup(const :ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`& tfDataPack)

Decorator **call** () function. Takes the lower decorator as a parameter. Moves the given class into a shared_ptr, which will be managed by the next decorator.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- tfDataPack

		- Lower Decorator



.. rubric:: Returns:

shared_ptr referencing data from this object

.. index:: pair: function; linkedEngineName
.. _doxid-class_transceiver_data_pack_interface_1a0857c957956587af7a92f981a26ec987:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::string& linkedEngineName() const

Get name of engine this transceiver is linked to.

.. index:: pair: function; isPrepocessing
.. _doxid-class_transceiver_data_pack_interface_1aa10d5676bd9cbcfd93afddda59e077fd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual bool isPrepocessing() const

Indicates if this is a preprocessing function.

.. index:: pair: function; runTf
.. _doxid-class_transceiver_data_pack_interface_1aebff4b55ef032d519cf1ee594ba76806:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual boost::python::object runTf(
		boost::python::tuple& args,
		boost::python::dict& kwargs
	)

Execute Transceiver Function. Base class will simply call runTf on _function.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Arguments for execution. Can be altered by any TransceiverDataPackInterfaces. Base class will only pass them along

	*
		- kwargs

		- Keyword arguments for execution. Can be altered by any TransceiverDataPackInterfaces. Base class will only pass them along



.. rubric:: Returns:

Returns result of :ref:`TransceiverFunction <doxid-class_transceiver_function>` execution.

.. index:: pair: function; updateRequestedDataPackIDs
.. _doxid-class_transceiver_data_pack_interface_1acb6252267d56aa7520bb5591ed69793d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` updateRequestedDataPackIDs(:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`&& datapackIDs = :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>`()) const

Appends its own datapack requests onto datapackIDs. Uses getRequestedDataPackIDs to check which IDs are requested by this datapack.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- datapackIDs

		- Container with datapack IDs that gets expanded



.. rubric:: Returns:

Returns datapackIDs, with own datapackIDs appended

.. index:: pair: function; getRequestedDataPackIDs
.. _doxid-class_transceiver_data_pack_interface_1aa2721b1c08b446bb21682f4c37e5cea0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` getRequestedDataPackIDs() const

Returns datapack IDs of this :ref:`DataPack <doxid-class_data_pack>` that should be requested from the engines. TODO: Make protected.

.. index:: pair: function; setTFInterpreter
.. _doxid-class_transceiver_data_pack_interface_1a271c6f176f41d1110e3316fe4653941a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static void setTFInterpreter(:ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`* interpreter)

Set global TF Interpreter. All Transceiver Functions will register themselves with it upon creation.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- interpreter

		- Interpreter to use

