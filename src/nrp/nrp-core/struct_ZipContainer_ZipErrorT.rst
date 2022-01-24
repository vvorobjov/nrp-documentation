.. index:: pair: struct; ZipContainer::ZipErrorT
.. _doxid-struct_zip_container_1_1_zip_error_t:

struct ZipContainer::ZipErrorT
==============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Wrapper for zip_error_t. Automatically initializes and cleans up struct. :ref:`More...<details-struct_zip_container_1_1_zip_error_t>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct ZipErrorT: public zip_error_t {
		// construction
	
		:ref:`ZipErrorT<doxid-struct_zip_container_1_1_zip_error_t_1a59b4603547108539f426d7649c829807>`();
		:ref:`ZipErrorT<doxid-struct_zip_container_1_1_zip_error_t_1abdbbcf54f408f1406774424b76aeacae>`(int ze);
	};
.. _details-struct_zip_container_1_1_zip_error_t:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Wrapper for zip_error_t. Automatically initializes and cleans up struct.

Construction
------------

.. index:: pair: function; ZipErrorT
.. _doxid-struct_zip_container_1_1_zip_error_t_1a59b4603547108539f426d7649c829807:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ZipErrorT()

Constructor. Sets up zip_error_t.

.. index:: pair: function; ZipErrorT
.. _doxid-struct_zip_container_1_1_zip_error_t_1abdbbcf54f408f1406774424b76aeacae:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ZipErrorT(int ze)

Constructor. Initializes zip_error_t with error code ze.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- ze

		- Error Code

