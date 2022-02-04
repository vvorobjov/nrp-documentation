.. index:: pair: struct; function_traits<std::function<R(Args...)>>
.. _doxid-structfunction__traits_3_01std_1_1function_3_01_r_07_args_8_8_8_08_4_01_4:

template struct function_traits<std::function<R(Args...)>>
==========================================================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <function_traits.h>
	
	template <typename R, typename ... Args>
	struct function_traits<std::function<R(Args...)>> {
		// typedefs
	
		typedef R :target:`result_t<doxid-structfunction__traits_3_01std_1_1function_3_01_r_07_args_8_8_8_08_4_01_4_1a3e8d8dfc50012c88247a79928620f021>`;
		typedef typename std::tuple_element<i, std::tuple<Args...>>::type :target:`arg_t<doxid-structfunction__traits_3_01std_1_1function_3_01_r_07_args_8_8_8_08_4_01_4_1a7cc04599b9b43e740586e7689bf6de71>`;

		// fields
	
		static const size_t :target:`nargs<doxid-structfunction__traits_3_01std_1_1function_3_01_r_07_args_8_8_8_08_4_01_4_1a2270df8fec073455c29c6b0461ebbbb0>` = sizeof...(Args);
	};
