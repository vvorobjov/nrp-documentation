.. index:: pair: struct; TransceiverDataPackInterfaceWrapper
.. _doxid-struct_transceiver_data_pack_interface_wrapper:

struct TransceiverDataPackInterfaceWrapper
==========================================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct TransceiverDataPackInterfaceWrapper:
	    public :ref:`TransceiverDataPackInterface<doxid-class_transceiver_data_pack_interface>`,
	    public boost::python::wrapper< TransceiverDataPackInterface > {
		// methods
	
		python::object :target:`runTf<doxid-struct_transceiver_data_pack_interface_wrapper_1af92691e802786d063a3bde436942625d>`(python::tuple& args, python::dict& kwargs);
		python::object :target:`defaultRunTf<doxid-struct_transceiver_data_pack_interface_wrapper_1a12648a8e48fb4cb0ea627722cc52c41f>`(python::tuple& args, python::dict& kwargs);
		virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :ref:`getRequestedDataPackIDs<doxid-struct_transceiver_data_pack_interface_wrapper_1af1363b3380b27079db83d75e12d6b31e>`() const;
		:ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :target:`defaultGetRequestedDataPackIDs<doxid-struct_transceiver_data_pack_interface_wrapper_1ace5b11bea1de1c7f31e5c4f8273651ac>`() const;
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

.. _details-struct_transceiver_data_pack_interface_wrapper:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; getRequestedDataPackIDs
.. _doxid-struct_transceiver_data_pack_interface_wrapper_1af1363b3380b27079db83d75e12d6b31e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` getRequestedDataPackIDs() const

Returns datapack IDs of this :ref:`DataPack <doxid-class_data_pack>` that should be requested from the engines. TODO: Make protected.

