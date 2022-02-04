.. index:: pair: struct; tuple_array
.. _doxid-structtuple__array:

template struct tuple_array
===========================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <functional_node.h>
	
	template <typename T, size_t N>
	struct tuple_array {
		// fields
	
		decltype(std::tuple_cat(std::tuple<T>(), typename tuple_array<T, N-1>::type())) typedef :target:`type<doxid-structtuple__array_1a6b43348538c40ae1e0f919063bfb9bd6>`;
	};
