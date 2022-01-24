.. index:: pair: class; FTILoop
.. _doxid-class_f_t_i_loop:

class FTILoop
=============

.. toctree::
	:hidden:

Overview
~~~~~~~~

Manages simulation loop. Runs physics and brain interface, and synchronizes them via Transfer Functions. :ref:`More...<details-class_f_t_i_loop>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <fti_loop.h>
	
	class FTILoop: public :ref:`PtrTemplates<doxid-class_ptr_templates>` {
	public:
		// construction
	
		:target:`FTILoop<doxid-class_f_t_i_loop_1ab9c643284e1a1f225f56fb6c83a54661>`();
		:target:`FTILoop<doxid-class_f_t_i_loop_1a4ec7cbb7cddaa132f18244732cf18e63>`(:ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>` config, :ref:`DataPackProcessor::engine_interfaces_t<doxid-class_data_pack_processor_1a269b18f42f5acedb594ad2912c795824>` engines);

		// methods
	
		void :ref:`initLoop<doxid-class_f_t_i_loop_1a985fac90b7663ea39391d095ff1c160b>`();
		void :ref:`resetLoop<doxid-class_f_t_i_loop_1ac0523f0fa4026b4b2c1ec622d5655465>`();
		void :ref:`shutdownLoop<doxid-class_f_t_i_loop_1a1ec08d246fbbd1003b43e5110fe4ef56>`();
		void :ref:`runLoop<doxid-class_f_t_i_loop_1adedc0d66c8e286e31c3b00dcd9ee46dc>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runLoopTime);
		:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`getSimTime<doxid-class_f_t_i_loop_1a61e3bc08a20c2a09e45858c1d17c2277>`() const;
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// typedefs
	
		typedef std::shared_ptr<T> :ref:`shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`;
		typedef std::shared_ptr<const T> :ref:`const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>`;
		typedef std::unique_ptr<T> :ref:`unique_ptr<doxid-class_ptr_templates_1a6d24e150817ba36df80ce3b603b7c665>`;
		typedef std::unique_ptr<const T> :ref:`const_unique_ptr<doxid-class_ptr_templates_1aef0eb44f9c386dbf0de54d0f5afac667>`;

.. _details-class_f_t_i_loop:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Manages simulation loop. Runs physics and brain interface, and synchronizes them via Transfer Functions.

Methods
-------

.. index:: pair: function; initLoop
.. _doxid-class_f_t_i_loop_1a985fac90b7663ea39391d095ff1c160b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void initLoop()

Initialize engines before running loop.

.. index:: pair: function; resetLoop
.. _doxid-class_f_t_i_loop_1ac0523f0fa4026b4b2c1ec622d5655465:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void resetLoop()

Reset engines of the loop.

.. index:: pair: function; shutdownLoop
.. _doxid-class_f_t_i_loop_1a1ec08d246fbbd1003b43e5110fe4ef56:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void shutdownLoop()

Shutdown engines.

.. index:: pair: function; runLoop
.. _doxid-class_f_t_i_loop_1adedc0d66c8e286e31c3b00dcd9ee46dc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void runLoop(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runLoopTime)

Runs a single loop step.

Runs simulation for a total of runLoopTime (in s)



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- timeStep

		- How long the single components should run (in seconds)

	*
		- runLoopTime

		- Time (in s) to run simulation. At end, will run TransceiverFunctions

.. index:: pair: function; getSimTime
.. _doxid-class_f_t_i_loop_1a61e3bc08a20c2a09e45858c1d17c2277:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` getSimTime() const

Get Simulation Time (in seconds)



.. rubric:: Returns:

Returns time passed in simulation (in seconds)

