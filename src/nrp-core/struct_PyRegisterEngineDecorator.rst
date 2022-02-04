.. index:: pair: struct; PyRegisterEngineDecorator
.. _doxid-struct_py_register_engine_decorator:

struct PyRegisterEngineDecorator
================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Decorator for engine script class. User can decorate their EngineScript class with this to register it with the server. :ref:`More...<details-struct_py_register_engine_decorator>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct PyRegisterEngineDecorator {
		// methods
	
		:ref:`PyEngineScript<doxid-class_py_engine_script>`& :ref:`pyCall<doxid-struct_py_register_engine_decorator_1a869deb54ddbee133240798cd8b73228b>`(python::object script);
	};
.. _details-struct_py_register_engine_decorator:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Decorator for engine script class. User can decorate their EngineScript class with this to register it with the server.

Methods
-------

.. index:: pair: function; pyCall
.. _doxid-struct_py_register_engine_decorator_1a869deb54ddbee133240798cd8b73228b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`PyEngineScript<doxid-class_py_engine_script>`& pyCall(python::object script)

**call** () function



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- script

		- Class derived from EngineScript



.. rubric:: Returns:

Returns ref to :ref:`PyEngineScript <doxid-class_py_engine_script>`

