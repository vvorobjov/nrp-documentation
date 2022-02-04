.. index:: pair: class; PythonGILLock
.. _doxid-class_python_g_i_l_lock:

class PythonGILLock
===================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Manages the Pyton GIL. Useful for threads. :ref:`More...<details-class_python_g_i_l_lock>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <python_interpreter_state.h>
	
	class PythonGILLock {
	public:
		// construction
	
		:ref:`PythonGILLock<doxid-class_python_g_i_l_lock_1a88460b07b885500410f8937689f24aca>`(PyGILState_STATE& state, const bool acquire = true);
		:target:`PythonGILLock<doxid-class_python_g_i_l_lock_1a1352a2ec9ebedcb38629958747e17a14>`(const PythonGILLock&);

		// methods
	
		PythonGILLock& :target:`operator =<doxid-class_python_g_i_l_lock_1af34826b6a18d555edb6d02e8a62fc156>` (const PythonGILLock&);
		void :ref:`acquire<doxid-class_python_g_i_l_lock_1a59fd0aea4019b4e88ebad83ae91280a1>`();
		void :ref:`release<doxid-class_python_g_i_l_lock_1a20e9ec7905b5ef132f1c5039c4d8becc>`();
		static bool :ref:`hasGIL<doxid-class_python_g_i_l_lock_1a1d23a636cef6047bdc6cdafaee408dfa>`();
	};
.. _details-class_python_g_i_l_lock:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Manages the Pyton GIL. Useful for threads.

Construction
------------

.. index:: pair: function; PythonGILLock
.. _doxid-class_python_g_i_l_lock_1a88460b07b885500410f8937689f24aca:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	PythonGILLock(PyGILState_STATE& state, const bool acquire = true)

Constructor. Acquires GIL if requested.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- state

		- GIL State

	*
		- acquire

		- Should GIL be acquired

Methods
-------

.. index:: pair: function; acquire
.. _doxid-class_python_g_i_l_lock_1a59fd0aea4019b4e88ebad83ae91280a1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void acquire()

Acquire GIL.

.. index:: pair: function; release
.. _doxid-class_python_g_i_l_lock_1a20e9ec7905b5ef132f1c5039c4d8becc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void release()

Release GIL.

.. index:: pair: function; hasGIL
.. _doxid-class_python_g_i_l_lock_1a1d23a636cef6047bdc6cdafaee408dfa:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static bool hasGIL()

Does this thread have the GIL?

