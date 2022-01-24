.. index:: pair: struct; FixedString
.. _doxid-struct_fixed_string:

template struct FixedString
===========================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <fixed_string.h>
	
	template <std::size_t N>
	struct FixedString {
		// fields
	
		static constexpr auto :target:`Length<doxid-struct_fixed_string_1a7465b2b3f55d90f070e8e4ddd8aec5a8>` = N;
		char :target:`m_data<doxid-struct_fixed_string_1afd83e475ea843a5cba7326fdcdebd232>`[N] {};

		// construction
	
		:target:`FixedString<doxid-struct_fixed_string_1ac384e2e55529a22d2046c7e6e52d5af7>`(const char(&) str[N]);
		:target:`FixedString<doxid-struct_fixed_string_1ad719e28875610d757e15c15046157fa6>`(const FixedString<N>& str);
	
		template <class T>
		:target:`FixedString<doxid-struct_fixed_string_1a5e6c8ee4f8ab6dad1ec09631c1074c56>`(T str);
	
		:target:`FixedString<doxid-struct_fixed_string_1a2f93e19a2246b5525efaee98efbf6925>`();

		// methods
	
		constexpr :target:`operator auto<doxid-struct_fixed_string_1a7cd83588f32ac0f90c1b0835a505acd8>` () const;
		constexpr :ref:`operator int<doxid-struct_fixed_string_1a8c1274ff62674340598be84e947b8ccf>` () const;
		constexpr :target:`operator std::string_view<doxid-struct_fixed_string_1a69985ec05862a2d78f4631b3e07b98cf>` () const;
		constexpr :target:`operator std::string<doxid-struct_fixed_string_1ab8565a24cfcd28c35c8b11103291e6d5>` () const;
		constexpr const char* :target:`data<doxid-struct_fixed_string_1aa179466cfba847799604672ecd50d56e>`() const;
	
		template <std::size_t M>
		constexpr bool :target:`compare<doxid-struct_fixed_string_1ae70d13123acaa06739facdb0f5a67bae>`(const char(&) other[M]) const;
	
		constexpr bool :target:`compare<doxid-struct_fixed_string_1a12a52db782099eb9413d066e09a50aba>`(const char*const str) const;
		constexpr bool :target:`compare<doxid-struct_fixed_string_1a65a89d682d593858fc98dafe89cc2d75>`(const std::string_view& str) const;
	};
.. _details-struct_fixed_string:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; operator int
.. _doxid-struct_fixed_string_1a8c1274ff62674340598be84e947b8ccf:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	constexpr operator int () const

This is only here so that the CLang LSP doesn't complain about template parameter deduction. Remove once this is implemented.

