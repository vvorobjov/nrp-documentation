.. index:: pair: struct; dataConverter
.. _doxid-structdata_converter:

template struct dataConverter
=============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Template for data conversion functions used in InputPorts upon new message arrival. :ref:`More...<details-structdata_converter>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <data_conversion.h>
	
	template <class T_IN, class T_OUT>
	struct dataConverter {
		// methods
	
		static void :target:`convert<doxid-structdata_converter_1a3b8419951a503701a1aebf045185849c>`(const T_IN*, T_OUT&);
	};
.. _details-structdata_converter:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Template for data conversion functions used in InputPorts upon new message arrival.

Concrete conversions are implemented as specializations

