.. index:: pair: class; TransceiverFunction
.. _doxid-class_transceiver_function:

class TransceiverFunction
=========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Holds a single transfer function decorator. :ref:`More...<details-class_transceiver_function>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <transceiver_function.h>
	
	class TransceiverFunction:
	    public :ref:`TransceiverDataPackInterface<doxid-class_transceiver_data_pack_interface>`,
	    public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// construction
	
		:ref:`TransceiverFunction<doxid-class_transceiver_function_1a0e881e24f93df171b05317eddfda5730>`(std::string linkedEngine, bool isPreprocessing);

		// methods
	
		virtual const std::string& :ref:`linkedEngineName<doxid-class_transceiver_function_1a42aad9886158c92a69c54964f90e6d43>`() const;
		virtual bool :ref:`isPrepocessing<doxid-class_transceiver_function_1a7b3a21d2aca3d6f11abf8c7852cdabcd>`() const;
		:ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :ref:`pySetup<doxid-class_transceiver_function_1a573476ca789803466f05bc5590126e1d>`(boost::python::object transceiverFunction);
	
		virtual boost::python::object :ref:`runTf<doxid-class_transceiver_function_1ab4d35ba46260fd8464874ab3147bf6ff>`(
			boost::python::tuple& args,
			boost::python::dict& kwargs
		);
	};

	// direct descendants

	class :ref:`PreprocessingFunction<doxid-class_preprocessing_function>`;

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

.. _details-class_transceiver_function:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Holds a single transfer function decorator.

Construction
------------

.. index:: pair: function; TransceiverFunction
.. _doxid-class_transceiver_function_1a0e881e24f93df171b05317eddfda5730:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	TransceiverFunction(std::string linkedEngine, bool isPreprocessing)

Constructor.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- linkedEngine

		- Name of linked engine. This TF will only be called whenever Said engine finishes a timestep

Methods
-------

.. index:: pair: function; linkedEngineName
.. _doxid-class_transceiver_function_1a42aad9886158c92a69c54964f90e6d43:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual const std::string& linkedEngineName() const

Get name of engine this transceiver is linked to.

.. index:: pair: function; isPrepocessing
.. _doxid-class_transceiver_function_1a7b3a21d2aca3d6f11abf8c7852cdabcd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual bool isPrepocessing() const

Indicates if this is a preprocessing function.

.. index:: pair: function; pySetup
.. _doxid-class_transceiver_function_1a573476ca789803466f05bc5590126e1d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` pySetup(boost::python::object transceiverFunction)

Decorator **call** () function. Takes the Transfer Function as a parameter. Moves this class into a shared_ptr.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- transceiverFunction

		- User defined TF



.. rubric:: Returns:

shared_ptr referencing data from this object

.. index:: pair: function; runTf
.. _doxid-class_transceiver_function_1ab4d35ba46260fd8464874ab3147bf6ff:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual boost::python::object runTf(
		boost::python::tuple& args,
		boost::python::dict& kwargs
	)

Execute the transfer function.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Python args

	*
		- kwargs

		- Python keywords



.. rubric:: Returns:

Returns result of TF

