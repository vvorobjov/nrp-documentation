.. index:: pair: enum; MsgPublishPolicy
.. _doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2:

enum OutputNodePolicies::MsgPublishPolicy
=========================================

Overview
~~~~~~~~

Defines how output nodes send stored msgs. :ref:`More...<details-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2>`

.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_node_policies.h>

	enum MsgPublishPolicy {
	    :ref:`SERIES<doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2a6ea52d77194fc390647024abd57f78b2>`,
	    :ref:`BATCH<doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2a438aa69f27e0c11c45e587422753a0e4>`,
	};

.. _details-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Defines how output nodes send stored msgs.

Enum Values
-----------

.. index:: pair: enumvalue; SERIES
.. _doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2a6ea52d77194fc390647024abd57f78b2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	SERIES

sends received msgs one by one

.. index:: pair: enumvalue; BATCH
.. _doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2a438aa69f27e0c11c45e587422753a0e4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	BATCH

sends all msgs received in a single batch

