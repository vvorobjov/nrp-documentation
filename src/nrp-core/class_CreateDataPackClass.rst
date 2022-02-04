.. index:: pair: class; CreateDataPackClass
.. _doxid-class_create_data_pack_class:

class CreateDataPackClass
=========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Singleton class. Used to create Nest datapacks in a manner that makes them accessible to the NRP. :ref:`More...<details-class_create_data_pack_class>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <create_datapack_class.h>
	
	class CreateDataPackClass {
	public:
		// construction
	
		:ref:`CreateDataPackClass<doxid-class_create_data_pack_class_1a822be32a750f56f74facde4f5f59683f>`(boost::python::dict nest, boost::python::dict devMap);

		// methods
	
		boost::python::object :ref:`createAndRegisterDataPack<doxid-class_create_data_pack_class_1ae1ebdd9cfededd3866de686f5ada3fe2>`(
			boost::python::tuple args,
			boost::python::dict kwargs
		);
	
		void :ref:`registerDataPack<doxid-class_create_data_pack_class_1aa736ec319982f23b5be8da6ffe7a9b19>`(
			boost::python::str devName,
			boost::python::object nodeCollection
		);
	
		boost::python::dict :ref:`pyDevMap<doxid-class_create_data_pack_class_1ae281d645aad8697c7b250edc60f7bafa>`();
		const boost::python::dict& :ref:`devMap<doxid-class_create_data_pack_class_1a64888b753ee795908d0f3eaf09b3f109>`() const;
	
		boost::python::object :ref:`createAndRegisterDataPack<doxid-class_create_data_pack_class_1ac7cc036968d3b6052bc0f7e84871cded>`(
			boost::python::tuple args,
			boost::python::dict kwargs
		);
	
		const boost::python::dict& :ref:`devMap<doxid-class_create_data_pack_class_1ae9779440a2369820229b4a7455d80dc9>`() const;
	
		static boost::python::object :ref:`pyCreateDataPack<doxid-class_create_data_pack_class_1a8862f566b5fd59fe04ebb7ce99ad49ce>`(
			boost::python::tuple args,
			boost::python::dict kwargs
		);
	
		static boost::python::object :ref:`pyRegisterDataPack<doxid-class_create_data_pack_class_1a373a098b914e1d36e24ff4be50d5c32a>`(
			boost::python::tuple args,
			boost::python::dict kwargs
		);
	
		static boost::python::object :ref:`pyCreateDataPack<doxid-class_create_data_pack_class_1a8a8e2ccbb4ccde0f935da938458f8d92>`(
			boost::python::tuple args,
			boost::python::dict kwargs
		);
	
		static boost::python::object :ref:`pyRegisterDataPack<doxid-class_create_data_pack_class_1aaafa20940c13dc0bb92a8b72c99d6084>`(
			boost::python::tuple args,
			boost::python::dict kwargs
		);
	};
.. _details-class_create_data_pack_class:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Singleton class. Used to create Nest datapacks in a manner that makes them accessible to the NRP.

Construction
------------

.. index:: pair: function; CreateDataPackClass
.. _doxid-class_create_data_pack_class_1a822be32a750f56f74facde4f5f59683f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	CreateDataPackClass(boost::python::dict nest, boost::python::dict devMap)

Constructor.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- nest

		- Nest Dict

	*
		- devMap

		- :ref:`DataPack <doxid-class_data_pack>` Mapping

Methods
-------

.. index:: pair: function; createAndRegisterDataPack
.. _doxid-class_create_data_pack_class_1ae1ebdd9cfededd3866de686f5ada3fe2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object createAndRegisterDataPack(
		boost::python::tuple args,
		boost::python::dict kwargs
	)

Create new Nest datapack and add it to devMap.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Python Args. First argument must be string with datapack label. This will be used to reference the datapack from the NRP side. All other args will be passed to nest.Create(...)

	*
		- kwargs

		- Python Keyword Args. Will be passed to nest.Create(...)



.. rubric:: Returns:

Returns either Nest NodeCollection (Nest 3.x) or python::tuple of nest GIDs (Nest 2.x)

.. index:: pair: function; registerDataPack
.. _doxid-class_create_data_pack_class_1aa736ec319982f23b5be8da6ffe7a9b19:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void registerDataPack(
		boost::python::str devName,
		boost::python::object nodeCollection
	)

Register an existing datapack.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- devName

		- NRP :ref:`DataPack <doxid-class_data_pack>` name

	*
		- nodeCollection

		- NodeCollection of datapack data

.. index:: pair: function; pyDevMap
.. _doxid-class_create_data_pack_class_1ae281d645aad8697c7b250edc60f7bafa:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::dict pyDevMap()

Python Call Function to get NRP datapack mapping.



.. rubric:: Returns:

Returns dict with datapack mapping

.. index:: pair: function; devMap
.. _doxid-class_create_data_pack_class_1a64888b753ee795908d0f3eaf09b3f109:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const boost::python::dict& devMap() const

Get datapack map.



.. rubric:: Returns:

Returns datapack map

.. index:: pair: function; createAndRegisterDataPack
.. _doxid-class_create_data_pack_class_1ac7cc036968d3b6052bc0f7e84871cded:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	boost::python::object createAndRegisterDataPack(
		boost::python::tuple args,
		boost::python::dict kwargs
	)

Create new Nest datapack and add it to devMap.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Python Args. First argument must be string with datapack label. This will be used to reference the datapack from the NRP side. All other args will be passed to nest.Create(...)

	*
		- kwargs

		- Python Keyword Args. Will be passed to nest.Create(...)



.. rubric:: Returns:

Returns either Nest NodeCollection (Nest 3.x) or python::tuple of nest GIDs (Nest 2.x)

.. index:: pair: function; devMap
.. _doxid-class_create_data_pack_class_1ae9779440a2369820229b4a7455d80dc9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const boost::python::dict& devMap() const

Get datapack map.



.. rubric:: Returns:

Returns datapack map

.. index:: pair: function; pyCreateDataPack
.. _doxid-class_create_data_pack_class_1a8862f566b5fd59fe04ebb7ce99ad49ce:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static boost::python::object pyCreateDataPack(
		boost::python::tuple args,
		boost::python::dict kwargs
	)

Python function to create datapack and register it in _devMap.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Python Args. args[0] is :ref:`CreateDataPackClass <doxid-class_create_data_pack_class>`, args[1] is the NRP datapack name. The remaining args will be passed to nest.Create(...)

	*
		- kwargs

		- Python Keyword Args. Will be passed to nest.Create(...)



.. rubric:: Returns:

Returns either Nest NodeCollection (Nest 3.x) or python::tuple of nest GIDs (Nest 2.x)

.. index:: pair: function; pyRegisterDataPack
.. _doxid-class_create_data_pack_class_1a373a098b914e1d36e24ff4be50d5c32a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static boost::python::object pyRegisterDataPack(
		boost::python::tuple args,
		boost::python::dict kwargs
	)

pyRegisterDataPack



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Python Args. args[0] is :ref:`CreateDataPackClass <doxid-class_create_data_pack_class>`, args[1] is the NRP datapack name, args[2] is a NodeCollection object

	*
		- kwargs

		- Python Kwargs. Not used

.. index:: pair: function; pyCreateDataPack
.. _doxid-class_create_data_pack_class_1a8a8e2ccbb4ccde0f935da938458f8d92:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static boost::python::object pyCreateDataPack(
		boost::python::tuple args,
		boost::python::dict kwargs
	)

Python function to create datapack and register it in _devMap.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Python Args. args[0] is :ref:`CreateDataPackClass <doxid-class_create_data_pack_class>`, args[1] is the NRP datapack name. The remaining args will be passed to nest.Create(...)

	*
		- kwargs

		- Python Keyword Args. Will be passed to nest.Create(...)



.. rubric:: Returns:

Returns either Nest NodeCollection (Nest 3.x) or python::tuple of nest GIDs (Nest 2.x)

.. index:: pair: function; pyRegisterDataPack
.. _doxid-class_create_data_pack_class_1aaafa20940c13dc0bb92a8b72c99d6084:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static boost::python::object pyRegisterDataPack(
		boost::python::tuple args,
		boost::python::dict kwargs
	)

pyRegisterDataPack



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- args

		- Python Args. args[0] is :ref:`CreateDataPackClass <doxid-class_create_data_pack_class>`, args[1] is the NRP datapack name, args[2] is a NodeCollection object

	*
		- kwargs

		- Python Kwargs. Not used

