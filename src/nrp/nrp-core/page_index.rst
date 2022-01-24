.. index:: pair: page; NRP Core
.. _doxid-indexpage:

NRP Core
========

The neurorobotics platform core (referred throughout this document as NRP-core) is the mechanism through which NRP users can implement simulations whereby multiple pieces of simulation software can coexist, synchronize their execution and exchange data in tightly ordered fashion. In previous versions of the NRP, NRP-core was referred to as the "Closed Loop Engine" (CLE), the task of which was to orchestrate the dialogue between the Gazebo robotic simulator and brain models implemented in NEST, Nengo, etc. For those of our users familiar with the CLE, NRP-core is a generalization of the latter, with new generic mechanisms provided to users who want to integrate new simulation engines into their NRP simulations. NRP-core is still built on the so-called Transfer Function framework, although the latter was adapted and renamed :ref:`Transceiver Functions <doxid-transceiver_function>` framework. This renaming is not only cosmetic: users familiar with the NRP up to v3.2 should indeed remain aware of some limited but meaningful evolutions between these two frameworks.

NRP-core is mostly written in C++, with the Transceiver Function framework relying on Python for better usability. It guarantees a fully deterministic execution of the simulation, provided every simulator used is itself deterministic, and works on the basis of controlled progression through time steps. Users should thus take note that event-based simulators may not be suitable for integration in NRP-core (to be analyzed on a case-by-case basis). Communications to and from NRP-core are indeed synchronous, and function calls are blocking; as such, the actual execution time of a simulation based on NRP-core will critically depend on the slowest simulator integrated therein.

To understand how to use and operate NRP-core, it is essential to be aware of some terminology used in the rest of this documentation. NRP-core communicates with other simulators or control processes by managing them as :ref:`simulation engines <doxid-engines>`. The term "engine" refers to an abstraction of the client-server architecture that is implemented for control and communication between NRP-core and every simulator (or process, or module, etc.) that is integrated into a given simulation. These engines exchange data in a standardised way, through objects that we refer to as :ref:`datapacks <doxid-datapacks>`. Defining the proper datapack for a given simulator in a given simulation is one of the most important steps in using the NRP with NRP-core. It is thus recommended that users take some time to familiarize themselves with this concept before using NRP-core and NRP versions built thereupon.

The documentation is organized in the following sections:

* :ref:`Getting started <doxid-getting_started>` : installation guide and walk-through of some simple experiments

* :ref:`Architecture overview <doxid-architecture_overview>` : description of the main components and concepts involved in the framework and how they interact with each other

* :ref:`Simulation Configuration <doxid-simulation_configuration>` : this section offer details on how experiments in NRP-core are configured

* :ref:`Engines: <doxid-nrp_engines>` list of supported engines as well as their configuration details

* :ref:`Guides <doxid-guides>` : guides to create your own experiments, integrate new engines or extend the existing ones and other things

Acknowledgments This work has received funding from the European Union’s Horizon 2020 Framework Programme for Research and Innovation under the Specific Grant Agreement No. 785907 (Human Brain Project SGA2) and under the Specific Grant Agreement No. 945539 (Human Brain Project SGA3).

License This work is licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at:

`http://www.apache.org/licenses/LICENSE-2.0 <http://www.apache.org/licenses/LICENSE-2.0>`__

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

The first version of NRP-Core was written by Michael Zechmair, and was later refactored and expanded by Eloy Retamino and Krzysztof Lebioda. Additional inputs were provided by Fabrice O. Morin, Ugo Albanese and Yuhong Huang.

.. toctree::
	:hidden:

	page_architecture_overview.rst
	page_nrp_engines.rst
	page_getting_started.rst
	page_guides.rst
	page_simulation_configuration.rst

.. rubric:: Related Pages:

|	:doc:`page_architecture_overview`
|		:doc:`page_datapacks`
|		:doc:`page_plugin_system`
|		:doc:`page_engines`
|		:doc:`page_event_loop`
|			:doc:`page_computational_graph`
|			:doc:`page_event_loop_configuration`
|			:doc:`page_python_graph`
|		:doc:`page_lifecycle_components`
|		:doc:`page_nrp_simulation`
|		:doc:`page_preprocessing_function`
|		:doc:`page_process_launcher`
|		:doc:`page_simulation_loop`
|		:doc:`page_sync_model_details`
|		:doc:`page_transceiver_function`
|	:doc:`page_nrp_engines`
|		:doc:`page_engine_comm`
|		:doc:`page_gazebo_engine`
|			:doc:`page_gazebo_datapacks`
|			:doc:`page_gazebo_plugins`
|		:doc:`page_nest_engine`
|			:doc:`page_nest_json`
|			:doc:`page_nest_server`
|		:doc:`page_opensim_engine`
|		:doc:`page_python_json_engine`
|	:doc:`page_getting_started`
|		:doc:`page_installation`
|		:doc:`page_running_example_exp`
|	:doc:`page_guides`
|		:doc:`page_tutorial_add_ros_msg_definition`
|		:doc:`page_tutorial_add_proto_definition`
|		:doc:`page_tutorial_engine_creation`
|			:doc:`page_tutorial_engine_creation_engine_cmake_example_explanation`
|		:doc:`page_engine_creation_template`
|		:doc:`page_tutorial_developer_guide`
|		:doc:`page_tutorial_helpful_info`
|		:doc:`page_python_engine_guide`
|	:doc:`page_simulation_configuration`
|		:doc:`page_engine_base_schema`
|		:doc:`page_json_schema`
|		:doc:`page_simulation_schema`
|		:doc:`page_transceiver_function_schema`


