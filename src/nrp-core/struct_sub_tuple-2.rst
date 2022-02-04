.. index:: pair: struct; sub_tuple<IDX1, std::tuple<Tpack...>, Tuple, IDX2>
.. _doxid-structsub__tuple_3_01_i_d_x1_00_01std_1_1tuple_3_01_tpack_8_8_8_01_4_00_01_tuple_00_01_i_d_x2_01_4:

template struct sub_tuple<IDX1, std::tuple<Tpack...>, Tuple, IDX2>
==================================================================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <functional_node_factory.h>
	
	template <size_t IDX1, typename... Tpack, typename Tuple, size_t IDX2>
	struct sub_tuple<IDX1, std::tuple<Tpack...>, Tuple, IDX2>: public :ref:`sub_tuple<doxid-structsub__tuple>` {
	};
