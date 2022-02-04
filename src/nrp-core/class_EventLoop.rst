.. index:: pair: class; EventLoop
.. _doxid-class_event_loop:

class EventLoop
===============

.. toctree::
	:hidden:

Overview
~~~~~~~~

Manages simulation loop. Runs physics and brain interface, and synchronizes them via Transfer Functions. :ref:`More...<details-class_event_loop>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <event_loop.h>
	
	class EventLoop {
	public:
		// construction
	
		:target:`EventLoop<doxid-class_event_loop_1a2eb3d9a9905743c87066f11253e60be2>`();
	
		:ref:`EventLoop<doxid-class_event_loop_1a6eef72dc16bd33ffa15c26e2997904f2>`(
			const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& graph_config,
			std::chrono::milliseconds timestep,
			bool ownGIL = true,
			bool spinROS = false
		);

		// methods
	
		void :ref:`runLoopOnce<doxid-class_event_loop_1a73bf10d6d1d378a871ef5ad38380c270>`();
		void :ref:`runLoopAsync<doxid-class_event_loop_1a4cbff5467f83d5e6fd0a67c4fb2e1086>`(std::chrono::milliseconds timeout = std::chrono::milliseconds(0));
		void :ref:`stopLoop<doxid-class_event_loop_1a660bd97a39a01dd15e527f9eee539047>`();
		bool :ref:`isRunning<doxid-class_event_loop_1aa5d546ac75355b3d114653a0659b9a65>`();
		void :ref:`waitForLoopEnd<doxid-class_event_loop_1a991476bde7fc75b2c98a073c020e1a75>`();
	};
.. _details-class_event_loop:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Manages simulation loop. Runs physics and brain interface, and synchronizes them via Transfer Functions.

Construction
------------

.. index:: pair: function; EventLoop
.. _doxid-class_event_loop_1a6eef72dc16bd33ffa15c26e2997904f2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	EventLoop(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& graph_config,
		std::chrono::milliseconds timestep,
		bool ownGIL = true,
		bool spinROS = false
	)

Constructor.

Methods
-------

.. index:: pair: function; runLoopOnce
.. _doxid-class_event_loop_1a73bf10d6d1d378a871ef5ad38380c270:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void runLoopOnce()

Run a single loop.

.. index:: pair: function; runLoopAsync
.. _doxid-class_event_loop_1a4cbff5467f83d5e6fd0a67c4fb2e1086:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void runLoopAsync(std::chrono::milliseconds timeout = std::chrono::milliseconds(0))

Run loop in a thread.

.. index:: pair: function; stopLoop
.. _doxid-class_event_loop_1a660bd97a39a01dd15e527f9eee539047:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void stopLoop()

Stop loop.

.. index:: pair: function; isRunning
.. _doxid-class_event_loop_1aa5d546ac75355b3d114653a0659b9a65:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool isRunning()

Returns true if the event loop is currently running, false otherwise.

.. index:: pair: function; waitForLoopEnd
.. _doxid-class_event_loop_1a991476bde7fc75b2c98a073c020e1a75:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void waitForLoopEnd()

Blocks execution until the loop reaches timeout.

