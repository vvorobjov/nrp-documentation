.. index:: pair: class; FileFinder
.. _doxid-class_file_finder:

class FileFinder
================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Find a file in a list of directories. :ref:`More...<details-class_file_finder>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <file_finder.h>
	
	class FileFinder {
	public:
		// methods
	
		static std::filesystem::path :ref:`findFile<doxid-class_file_finder_1a3c9d60b2a6a6faa4cbb734caf6f3edf1>`(
			const std::string& fileName,
			const std::vector<std::filesystem::path>& searchDirectories
		);
	
		std::filesystem::path :ref:`operator ()<doxid-class_file_finder_1adfb095e72bf7dca062628bd199fa2448>` (
			const std::string& fileName,
			const std::vector<std::filesystem::path>& searchDirectories
		);
	};
.. _details-class_file_finder:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Find a file in a list of directories.

Methods
-------

.. index:: pair: function; findFile
.. _doxid-class_file_finder_1a3c9d60b2a6a6faa4cbb734caf6f3edf1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static std::filesystem::path findFile(
		const std::string& fileName,
		const std::vector<std::filesystem::path>& searchDirectories
	)

Find first instance of file in searchDirectories.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fileName

		- Filename to find

	*
		- searchDirectories

		- Directories under which to search for file



.. rubric:: Returns:

Returns Path to fileName. Empty if not found

.. index:: pair: function; operator()
.. _doxid-class_file_finder_1adfb095e72bf7dca062628bd199fa2448:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::filesystem::path operator () (
		const std::string& fileName,
		const std::vector<std::filesystem::path>& searchDirectories
	)

Find first instance of file in searchDirectories.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fileName

		- Filename to find

	*
		- searchDirectories

		- Directories under which to search for file



.. rubric:: Returns:

Returns Path to fileName. Empty if not found

