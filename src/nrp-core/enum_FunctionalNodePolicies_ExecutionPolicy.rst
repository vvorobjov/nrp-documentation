.. index:: pair: enum; ExecutionPolicy
.. _doxid-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87:

enum FunctionalNodePolicies::ExecutionPolicy
============================================

Overview
~~~~~~~~

Possible execution policies for this node. :ref:`More...<details-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87>`

.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_node_policies.h>

	enum ExecutionPolicy {
	    :ref:`ALWAYS<doxid-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87ae7d3c14d40ccda0b6615a55e4d52eeec>`,
	    :ref:`ON_NEW_INPUT<doxid-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87a22a621ec60c2b27f4a0dcdc54d19948e>`,
	};

.. _details-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Possible execution policies for this node.

Enum Values
-----------

.. index:: pair: enumvalue; ALWAYS
.. _doxid-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87ae7d3c14d40ccda0b6615a55e4d52eeec:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ALWAYS

the node is always executed when 'compute' is called

.. index:: pair: enumvalue; ON_NEW_INPUT
.. _doxid-namespace_functional_node_policies_1a3f872dbefb885b0dca2745e76e002b87a22a621ec60c2b27f4a0dcdc54d19948e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ON_NEW_INPUT

the node is executed only if at least one of its inputs have a fresh value

