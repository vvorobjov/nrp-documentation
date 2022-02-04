.. index:: pair: struct; ZipWrapper
.. _doxid-struct_zip_wrapper:

struct ZipWrapper
=================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct ZipWrapper {
		// construction
	
		:target:`ZipWrapper<doxid-struct_zip_wrapper_1a412700517e228dac8766ece160c0b6cd>`(zip_t* zip);

		// methods
	
		zip_t* :target:`release<doxid-struct_zip_wrapper_1a93d44675639125d06da94957bc1360ce>`();
		void :target:`closeAndSaveZip<doxid-struct_zip_wrapper_1ac42b57923a00ba25a8da0576cd526e3a>`();
		:target:`operator zip_t *<doxid-struct_zip_wrapper_1a9b59bfb4ca101eb4950a581dc5b54464>` () const;
	};
