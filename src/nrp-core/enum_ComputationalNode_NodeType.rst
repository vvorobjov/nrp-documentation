.. index:: pair: enum; NodeType
.. _doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0:

enum ComputationalNode::NodeType
================================

Overview
~~~~~~~~

All the possible node types. :ref:`More...<details-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>`

.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <computational_node.h>

	enum NodeType {
	    :ref:`Input<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0a6f802cd82eb406c67acf68827a7f12c6>`,
	    :ref:`Output<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0af37c40dd3c9966f6c0762f208eb53d81>`,
	    :ref:`Functional<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0a74fee64f5e91b5ae74add89b9823e810>`,
	};

.. _details-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

All the possible node types.

Enum Values
-----------

.. index:: pair: enumvalue; Input
.. _doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0a6f802cd82eb406c67acf68827a7f12c6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	Input

only can be source in edges

.. index:: pair: enumvalue; Output
.. _doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0af37c40dd3c9966f6c0762f208eb53d81:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	Output

only can be target in edges

.. index:: pair: enumvalue; Functional
.. _doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0a74fee64f5e91b5ae74add89b9823e810:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	Functional

can be source and target, ie. can receive inputs and send outputs

