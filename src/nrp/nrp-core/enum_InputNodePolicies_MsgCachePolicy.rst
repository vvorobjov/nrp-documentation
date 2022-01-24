.. index:: pair: enum; MsgCachePolicy
.. _doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5:

enum InputNodePolicies::MsgCachePolicy
======================================

Overview
~~~~~~~~

Defines input node message cache behavior. :ref:`More...<details-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5>`

.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_node_policies.h>

	enum MsgCachePolicy {
	    :ref:`CLEAR_CACHE<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5ad06cd89641ba0a486411641ef8cf9d15>`,
	    :ref:`KEEP_CACHE<doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5a0fe86060e6b2cc716b117185f3e44ae9>`,
	};

.. _details-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Defines input node message cache behavior.

Enum Values
-----------

.. index:: pair: enumvalue; CLEAR_CACHE
.. _doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5ad06cd89641ba0a486411641ef8cf9d15:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	CLEAR_CACHE

if no new msg arrives, stored msgs are replaced by nullptr in the next cycle

.. index:: pair: enumvalue; KEEP_CACHE
.. _doxid-namespace_input_node_policies_1a6e6c639f025a1af2a05b3f20e6b207a5a0fe86060e6b2cc716b117185f3e44ae9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	KEEP_CACHE

if no new msg arrives, stored msgs are kept

