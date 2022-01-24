.. index:: pair: namespace; NGraph
.. _doxid-namespace_n_graph:

namespace NGraph
================

.. toctree::
	:hidden:

	class_NGraph_tGraph.rst

Overview
~~~~~~~~

A mathematical graph object: a simple, directed, connected graph, where nodes are of arbitrary type (colores, cities, names, etc.)

Operations for adding and removing edges and vertices, together with functions for finding neighbors, subgraphs, and other properties are included. :ref:`More...<details-namespace_n_graph>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	namespace NGraph {

	// typedefs

	typedef :ref:`tGraph<doxid-class_n_graph_1_1t_graph>`<unsigned int> :target:`Graph<doxid-namespace_n_graph_1a397e01b6d7caaaf603e67c1a87f11048>`;
	typedef :ref:`tGraph<doxid-class_n_graph_1_1t_graph>`<int> :target:`iGraph<doxid-namespace_n_graph_1a00343a4aa68bb7a338a89a778942701f>`;
	typedef :ref:`tGraph<doxid-class_n_graph_1_1t_graph>`<std::string> :target:`sGraph<doxid-namespace_n_graph_1a7b2d8922d00fae3af037b172b39fd524>`;

	// classes

	template <typename T>
	class :ref:`tGraph<doxid-class_n_graph_1_1t_graph>`;

	// global functions

	template <typename T>
	std::istream& :target:`operator >><doxid-namespace_n_graph_1a619633fcec087680ab8d90510eade326>` (std::istream& s, :ref:`tGraph<doxid-class_n_graph_1_1t_graph>`<T>& G);

	template <typename T>
	std::ostream& :target:`operator <<<doxid-namespace_n_graph_1a8118891761ec5e37cfd25efa666c73d8>` (
		std::ostream& s,
		const :ref:`tGraph<doxid-class_n_graph_1_1t_graph>`<T>& G
	);

	} // namespace NGraph
.. _details-namespace_n_graph:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

A mathematical graph object: a simple, directed, connected graph, where nodes are of arbitrary type (colores, cities, names, etc.)

Operations for adding and removing edges and vertices, together with functions for finding neighbors, subgraphs, and other properties are included.

Example:

.. code-block:: none


	enum color {blue, green, red, yellow, pink, black};
	tGraph<color> A;

.. code-block:: none

	A.insert_edge(blue, red);
	A.insert_edge(yellow, blue);
	A.insert_edge(blue, black);

.. code-block:: none

	tGraph<color> B(A), S=A.subgraph(blue);

.. code-block:: none

