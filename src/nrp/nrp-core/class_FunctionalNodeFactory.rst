.. index:: pair: class; FunctionalNodeFactory
.. _doxid-class_functional_node_factory:

class FunctionalNodeFactory
===========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Creates an instance of :ref:`FunctionalNode <doxid-class_functional_node>` with the right template given a function signature. :ref:`More...<details-class_functional_node_factory>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <functional_node_factory.h>
	
	class FunctionalNodeFactory {
	public:
		// methods
	
		template <size_t in_n, size_t out_n, typename... args>
		static auto :ref:`create<doxid-class_functional_node_factory_1ad720a08abf9d171f7f79cb5f06ac9a21>`(
			const std::string& id,
			std::function<void(args...)> f
		);
	};
.. _details-class_functional_node_factory:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Creates an instance of :ref:`FunctionalNode <doxid-class_functional_node>` with the right template given a function signature.

It hides all the complexity related with specifying correctly :ref:`FunctionalNode <doxid-class_functional_node>` template and performs function signature checking ensuring that the instantiated :ref:`FunctionalNode <doxid-class_functional_node>` is correct

Methods
-------

.. index:: pair: function; create
.. _doxid-class_functional_node_factory_1ad720a08abf9d171f7f79cb5f06ac9a21:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <size_t in_n, size_t out_n, typename... args>
	static auto create(
		const std::string& id,
		std::function<void(args...)> f
	)

Instantiates a :ref:`FunctionalNode <doxid-class_functional_node>`.

It takes as template parameters the number of inputs 'in_n' and outputs 'out_n' and a parameter pack containing the function arguments

