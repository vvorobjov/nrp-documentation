.. index:: pair: class; PythonInterpreterState
.. _doxid-class_python_interpreter_state:

class PythonInterpreterState
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Initializes the python interpreter as well as python threading. :ref:`More...<details-class_python_interpreter_state>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <python_interpreter_state.h>
	
	class PythonInterpreterState {
	public:
		// construction
	
		:ref:`PythonInterpreterState<doxid-class_python_interpreter_state_1a799af554f495ac4d81faa40e464d6be3>`(
			int argc,
			const char*const* argv,
			bool allowThreads = false
		);
	
		:ref:`PythonInterpreterState<doxid-class_python_interpreter_state_1a06aae17043ccf5217437890d2d20f89c>`(
			int argc,
			const std::vector<const char*>& argv,
			bool allowThreads = false
		);
	
		:ref:`PythonInterpreterState<doxid-class_python_interpreter_state_1ae88d7497a8d4524a97a8ee3f46bcace3>`(bool allowThreads = false);

		// methods
	
		void :ref:`allowThreads<doxid-class_python_interpreter_state_1a1fa41a1874254f4669dfe24542a9200c>`();
		bool :ref:`threadsAllowed<doxid-class_python_interpreter_state_1adab4aee0b85faf027771d494148dc2ae>`() const;
		void :ref:`endAllowThreads<doxid-class_python_interpreter_state_1a28d06b48040138537c6f20d812387ead>`();
	};
.. _details-class_python_interpreter_state:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Initializes the python interpreter as well as python threading.

Construction
------------

.. index:: pair: function; PythonInterpreterState
.. _doxid-class_python_interpreter_state_1a799af554f495ac4d81faa40e464d6be3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	PythonInterpreterState(
		int argc,
		const char*const* argv,
		bool allowThreads = false
	)

Constructor. Initializes Python with the given start parameters, enables threading, and releases GIL.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- argc

		- :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` 's argc

	*
		- argv

		- :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` 's argv

.. index:: pair: function; PythonInterpreterState
.. _doxid-class_python_interpreter_state_1a06aae17043ccf5217437890d2d20f89c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	PythonInterpreterState(
		int argc,
		const std::vector<const char*>& argv,
		bool allowThreads = false
	)

Constructor. Initializes Python with the given start parameters, enables threading, and releases GIL.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- argc

		- :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` 's argc

	*
		- argv

		- :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` 's argv

.. index:: pair: function; PythonInterpreterState
.. _doxid-class_python_interpreter_state_1ae88d7497a8d4524a97a8ee3f46bcace3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	PythonInterpreterState(bool allowThreads = false)

Constructor. Initializes Python with the no start parameters, enables threading, and releases GIL.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- argc

		- :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` 's argc

	*
		- argv

		- :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` 's argv

Methods
-------

.. index:: pair: function; allowThreads
.. _doxid-class_python_interpreter_state_1a1fa41a1874254f4669dfe24542a9200c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void allowThreads()

Allow execution of other threads. If this is set, main thread may not execute python code.

.. index:: pair: function; threadsAllowed
.. _doxid-class_python_interpreter_state_1adab4aee0b85faf027771d494148dc2ae:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool threadsAllowed() const

Are threads currently allowed?



.. rubric:: Returns:

Returns true if allowed, false otherwise

.. index:: pair: function; endAllowThreads
.. _doxid-class_python_interpreter_state_1a28d06b48040138537c6f20d812387ead:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void endAllowThreads()

Halt other threads from executin. This is required if python code should be executed in the main thread.

