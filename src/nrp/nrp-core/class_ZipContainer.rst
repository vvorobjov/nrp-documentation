.. index:: pair: class; ZipContainer
.. _doxid-class_zip_container:

class ZipContainer
==================

.. toctree::
	:hidden:

	struct_ZipContainer_ZipErrorT.rst
	struct_ZipContainer_ZipFileWrapper.rst

Overview
~~~~~~~~

Zip Container Structure. Based on libzip. :ref:`More...<details-class_zip_container>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <zip_container.h>
	
	class ZipContainer {
	public:
		// structs
	
		struct :ref:`ZipErrorT<doxid-struct_zip_container_1_1_zip_error_t>`;
		struct :ref:`ZipFileWrapper<doxid-struct_zip_container_1_1_zip_file_wrapper>`;

		// construction
	
		:ref:`ZipContainer<doxid-class_zip_container_1a4a79fecb0f4979c1cf6ed8a474cb0071>`(std::string&& data);
		:ref:`ZipContainer<doxid-class_zip_container_1a19b2a452b40f642b37362ad2ae88f74f>`(std::vector<uint8_t>&& data);
		:ref:`ZipContainer<doxid-class_zip_container_1a4b0af89bdb9ef9a82d92fb1632852924>`(const std::string& path, bool readOnly, bool saveOnDestruct);

		// methods
	
		std::vector<uint8_t> :ref:`getCompressedData<doxid-class_zip_container_1a863e543c4395278da44460b97a20f16f>`() const;
		void :ref:`extractZipFiles<doxid-class_zip_container_1a16bbe2acdb49f021ae2854c39b810e4c>`(std::string path) const;
		void :ref:`saveToDestination<doxid-class_zip_container_1ad64082fc764c302e10ea001da64bd0aa>`(const std::string& dest) const;
	
		static ZipContainer :ref:`compressPath<doxid-class_zip_container_1a7bb1bd118d5f58787416ffe34e5731c7>`(
			const std::filesystem::path& path,
			bool keepRelDirStruct = false
		);
	};
.. _details-class_zip_container:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Zip Container Structure. Based on libzip.

Construction
------------

.. index:: pair: function; ZipContainer
.. _doxid-class_zip_container_1a4a79fecb0f4979c1cf6ed8a474cb0071:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ZipContainer(std::string&& data)

Constructor. Takes a string argument. This is mainly used for Pistache data receiving.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Zip File Data. Note: Will use entire data.capacity() as ZIP file array, not just data.size()

	*
		- Throws

		- std::logic_error on failure

.. index:: pair: function; ZipContainer
.. _doxid-class_zip_container_1a19b2a452b40f642b37362ad2ae88f74f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ZipContainer(std::vector<uint8_t>&& data)

Constructor. Initializes zip_t.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- data

		- Zip data buffer

	*
		- Throws

		- std::logic_error on failure

.. index:: pair: function; ZipContainer
.. _doxid-class_zip_container_1a4b0af89bdb9ef9a82d92fb1632852924:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ZipContainer(const std::string& path, bool readOnly, bool saveOnDestruct)

Constructor. Loads data from file at path.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- path

		- Path to Zip Archive

	*
		- readOnly

		- Should archive be opened in read-only mode

	*
		- saveOnDestruct

		- Should the archive be saved automatically on destruct

Methods
-------

.. index:: pair: function; getCompressedData
.. _doxid-class_zip_container_1a863e543c4395278da44460b97a20f16f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::vector<uint8_t> getCompressedData() const

Get zip archive's compressed data.



.. rubric:: Returns:

Returns compressed data

.. index:: pair: function; extractZipFiles
.. _doxid-class_zip_container_1a16bbe2acdb49f021ae2854c39b810e4c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void extractZipFiles(std::string path) const

Extract Zip Files and store them under path.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- path

		- Path to extraction directory

	*
		- Throws

		- std::logic_error on fail

.. index:: pair: function; saveToDestination
.. _doxid-class_zip_container_1ad64082fc764c302e10ea001da64bd0aa:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void saveToDestination(const std::string& dest) const

Save Archive to storage.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- dest

		- File Name

	*
		- Throws

		- std::logic_error on fail

.. index:: pair: function; compressPath
.. _doxid-class_zip_container_1a7bb1bd118d5f58787416ffe34e5731c7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static ZipContainer compressPath(
		const std::filesystem::path& path,
		bool keepRelDirStruct = false
	)

Compress files and directories under path.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- path

		- Path to directory that should be compressed

	*
		- keepRelDirStruct

		- Should the created zip archive keep the relative directory structure to path. If false, will save files inside path directly

	*
		- Throws

		- std::logic_error on fail



.. rubric:: Returns:

Returns :ref:`ZipContainer <doxid-class_zip_container>` with compressed contents

