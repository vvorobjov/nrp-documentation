.. index:: pair: struct; ZipContainer::ZipFileWrapper
.. _doxid-struct_zip_container_1_1_zip_file_wrapper:

struct ZipContainer::ZipFileWrapper
===================================

.. toctree::
	:hidden:

Zip File Wrapper. Automatically closes file descriptor on desctruct.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct ZipFileWrapper {
		// construction
	
		:target:`ZipFileWrapper<doxid-struct_zip_container_1_1_zip_file_wrapper_1a86f0f8361d3856a8f66712ab801f5fea>`(zip_file_t* zFile);

		// methods
	
		:target:`operator zip_file_t *<doxid-struct_zip_container_1_1_zip_file_wrapper_1a8f0d0408c41748b0f3c7696cd368b56b>` ();
	};
