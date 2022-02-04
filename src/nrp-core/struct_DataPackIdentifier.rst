.. index:: pair: struct; DataPackIdentifier
.. _doxid-struct_data_pack_identifier:

struct DataPackIdentifier
=========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Identifies a single datapack. :ref:`More...<details-struct_data_pack_identifier>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <datapack_interface.h>
	
	struct DataPackIdentifier {
		// fields
	
		std::string :ref:`Name<doxid-struct_data_pack_identifier_1a4503921eb790287b4934104fe19d870b>`;
		std::string :ref:`EngineName<doxid-struct_data_pack_identifier_1a0f52d05427bba45a3bc49a6aa690d2f7>`;
		std::string :ref:`Type<doxid-struct_data_pack_identifier_1a39e482341dca27cee33a6d7d78f99605>`;

		// construction
	
		:target:`DataPackIdentifier<doxid-struct_data_pack_identifier_1a3fc1457461f957712780528eba7492ab>`();
	
		:target:`DataPackIdentifier<doxid-struct_data_pack_identifier_1aaf646cdaf3c8a21abfa9306fd469d616>`(
			const std::string& _name,
			const std::string& _engineName,
			const std::string& _type
		);
	
		:target:`DataPackIdentifier<doxid-struct_data_pack_identifier_1aaea4c3054e416a2d0f055a23379ee2bd>`(
			std::string&& _name,
			std::string&& _engineName,
			std::string&& _type
		);

		// methods
	
		bool :target:`operator ==<doxid-struct_data_pack_identifier_1a25283215a125788b85115eb122faa79d>` (const DataPackIdentifier& rhs) const;
		bool :target:`operator <<doxid-struct_data_pack_identifier_1a10806b38122f70e5f96a35366cafd307>` (const DataPackIdentifier& rhs) const;
	};
.. _details-struct_data_pack_identifier:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Identifies a single datapack.

Fields
------

.. index:: pair: variable; Name
.. _doxid-struct_data_pack_identifier_1a4503921eb790287b4934104fe19d870b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string Name

:ref:`DataPack <doxid-class_data_pack>` Name. Used by simulator to identify source/sink of datapack.

.. index:: pair: variable; EngineName
.. _doxid-struct_data_pack_identifier_1a0f52d05427bba45a3bc49a6aa690d2f7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string EngineName

Corresponding engine.

.. index:: pair: variable; Type
.. _doxid-struct_data_pack_identifier_1a39e482341dca27cee33a6d7d78f99605:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string Type

:ref:`DataPack <doxid-class_data_pack>` Type.

