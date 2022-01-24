.. index:: pair: enum; MsgPublishPolicy
.. _doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035:

enum InputNodePolicies::MsgPublishPolicy
========================================

Overview
~~~~~~~~

Defines how an input node publish stored msgs. :ref:`More...<details-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035>`

.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_node_policies.h>

	enum MsgPublishPolicy {
	    :ref:`LAST<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035add704bbffa58fee7b947ac8c26c4e2f4>`,
	    :ref:`ALL<doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035a9dba44949f3309ed09008d2505fd59a9>`,
	};

.. _details-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Defines how an input node publish stored msgs.

Enum Values
-----------

.. index:: pair: enumvalue; LAST
.. _doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035add704bbffa58fee7b947ac8c26c4e2f4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	LAST

only sends the last msg received

.. index:: pair: enumvalue; ALL
.. _doxid-namespace_input_node_policies_1ae65f9d4505207aa68b30fb0419c73035a9dba44949f3309ed09008d2505fd59a9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ALL

sends all msgs received since last 'compute' call

