.. index:: pair: class; GazeboStepController
.. _doxid-class_gazebo_step_controller:

class GazeboStepController
==========================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Controlls execution of Gazebo steps. Will be inherited by a Gazebo WorldPlugin. :ref:`More...<details-class_gazebo_step_controller>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <gazebo_step_controller.h>
	
	class GazeboStepController {
	public:
		// methods
	
		virtual :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :target:`runLoopStep<doxid-class_gazebo_step_controller_1a173267733547c908dce872bc0159e3e2>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` timeStep) = 0;
		virtual bool :target:`finishWorldLoading<doxid-class_gazebo_step_controller_1a31d62785aeb1635965a8875815fafa85>`() = 0;
		virtual bool :target:`resetWorld<doxid-class_gazebo_step_controller_1a0deeb096028c0223e0051e95e216b3b5>`() = 0;
	};
.. _details-class_gazebo_step_controller:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Controlls execution of Gazebo steps. Will be inherited by a Gazebo WorldPlugin.

Controls execution of Gazebo steps. Will be inherited by a Gazebo WorldPlugin.

