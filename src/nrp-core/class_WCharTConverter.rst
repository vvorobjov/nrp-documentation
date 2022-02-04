.. index:: pair: class; WCharTConverter
.. _doxid-class_w_char_t_converter:

class WCharTConverter
=====================

.. toctree::
	:hidden:

Converts and stores an char\*[] to wchar\*[]. Used mainly to convert argv from char\*[] to wchar\*[] for Python Initialization.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <wchar_t_converter.h>
	
	class WCharTConverter {
	public:
		// construction
	
		:target:`WCharTConverter<doxid-class_w_char_t_converter_1a2aedf85bd50c96792c65fa4d54dcba1f>`(int argc, const char*const* argv);

		// methods
	
		wchar_t** :target:`getWCharTPointers<doxid-class_w_char_t_converter_1a4eeb0a2bff0749b9bcf8aeb73ec62707>`();
		int :target:`getWCharSize<doxid-class_w_char_t_converter_1ad1585c77f789bd2b5ec8feef11c39e35>`() const;
	};
