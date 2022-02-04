.. index:: pair: class; EngineDataPack
.. _doxid-class_engine_data_pack:

class EngineDataPack
====================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Class for input datapacks for transceiver functions, mapped to :ref:`EngineDataPack <doxid-class_engine_data_pack>` python decorator. :ref:`More...<details-class_engine_data_pack>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <from_engine_datapack.h>
	
	class EngineDataPack: public :ref:`TransceiverDataPackInterface<doxid-class_transceiver_data_pack_interface>` {
	public:
		// construction
	
		:target:`EngineDataPack<doxid-class_engine_data_pack_1a868a7d139ebe58b3c39b52ff60633d8d>`(
			const std::string& keyword,
			const :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`& datapackID,
			bool isPreprocessed
		);

		// methods
	
		virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :ref:`getRequestedDataPackIDs<doxid-class_engine_data_pack_1ab819dd9ff62202b35e5544b7f5bc9762>`() const;
	
		virtual boost::python::object :ref:`runTf<doxid-class_engine_data_pack_1a509037eecb9f2dfde28b5718355db28b>`(
			boost::python::tuple& args,
			boost::python::dict& kwargs
		);
	};

	// direct descendants

	class :ref:`PreprocessedDataPack<doxid-class_preprocessed_data_pack>`;

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

.. _details-class_engine_data_pack:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Class for input datapacks for transceiver functions, mapped to :ref:`EngineDataPack <doxid-class_engine_data_pack>` python decorator.

Methods
-------

.. index:: pair: function; getRequestedDataPackIDs
.. _doxid-class_engine_data_pack_1ab819dd9ff62202b35e5544b7f5bc9762:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` getRequestedDataPackIDs() const

Returns datapack IDs of this :ref:`DataPack <doxid-class_data_pack>` that should be requested from the engines. TODO: Make protected.

.. index:: pair: function; runTf
.. _doxid-class_engine_data_pack_1a509037eecb9f2dfde28b5718355db28b:

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

