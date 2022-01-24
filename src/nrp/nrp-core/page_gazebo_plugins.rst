.. index:: pair: page; Gazebo Plugins
.. _doxid-gazebo_plugins:

Gazebo Plugins
==============

A set of plugins is provided to integrate gazebo in a NRP-core simulation. NRPGazeboCommunicationPlugin register the engine with the :ref:`SimulationManager <doxid-class_simulation_manager>` and handle control requests for advancing the gazebo simulation or shutting it down. Its use is mandatory in order to run the engine.

The rest of plugins handle datapack communication for a particular datapack type. Each of them allows to register datapacks of a concrete type which can then be accessed in TransceiverFunctions. From the list of gazebo_datapacks :ref:`supported datapacks <doxid-gazebo_datapacks>`, GazeboJointDataPack is the only one which can be used to affect the gazebo simulation, as explained below. The others, though they can be sent to the gazebo engine, will have no effect. That is, they are only useful for getting information from the simulation.

gRPC and JSON versions are provided for each of the plugins described below. Both versions behaves the same and are configured in the same way. From a user perspective they differ only in their names. gRPC plugins are named ``NRPGazeboGrpcSuffixName`` and JSON plugins ``NRPGazeboJSONSuffixName``. For simplicity, the documentation below refers only to the gRPC versions.



.. _doxid-gazebo_plugins_1NRPGazeboGrpcCommunicationPlugin:

NRPGazeboGrpcCommunicationPlugin
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

General communication plugin. Sets up a gRPC server and waits for NRP commands

.. ref-code-block:: cpp

	<world>
	...
	    <plugin name="nrp_world_plugin" filename="NRPGazeboGrpcWorldPlugin.so"/>
	...
	</world>





.. _doxid-gazebo_plugins_1NRPGazeboGrpcCameraPlugin:

NRPGazeboGrpcCameraPlugin
~~~~~~~~~~~~~~~~~~~~~~~~~

Adds a GazeboCameraDataPack datapack. The example below registers a datapack called 'camera_datapack'. This can then be accessed by TransceiverFunctions under that name

.. ref-code-block:: cpp

	<sensor name='camera' type='camera'>
	    ...
	    <plugin name='camera_datapack' filename='NRPGazeboGrpcCameraControllerPlugin.so'/>
	    ...
	</sensor>





.. _doxid-gazebo_plugins_1NRPGazeboGrpcLinkPlugin:

NRPGazeboGrpcLinkPlugin
~~~~~~~~~~~~~~~~~~~~~~~

Adds GazeboLinkDataPack datapacks for each link in the given model. The example below registers four datapacks under the name of their respective links names

.. ref-code-block:: cpp

	<model>
	    ...
	    <plugin name="link_plugin" filename="NRPGazeboGrpcLinkControllerPlugin.so" />
	    ...
	    <link name="back_left_link">...</link>
	    <link name="back_right_link">...</link>
	    <link name="front_left_link">...</link>
	    <link name="front_right_link">...</link>
	    ...
	</model>





.. _doxid-gazebo_plugins_1NRPGazeboGrpcJointPlugin:

NRPGazeboGrpcJointPlugin
~~~~~~~~~~~~~~~~~~~~~~~~

Adds GazeboJointDataPack datapacks. In this case only those joints that are explicitly named in the plugin will be registered and made available to the NRP. The example below registers four datapacks under the name of their respective joints.

In contrast to the plugins described above, when using NRPGazeboGrpcJointPlugin datapacks can be used to set a target state for the referenced joint. For each of the joint specified in the plugin configuration a PID controller is set. Incoming datapacks are used to adjust the controller target value. Two target types are supported: **position** and **velocity**.

.. ref-code-block:: cpp

	<model>
	    ...
	    <joint name="back_left_joint">...</joint>
	    <joint name="back_right_joint">...</joint>
	    <joint name="front_left_joint">...</joint>
	    <joint name="front_right_joint">...</joint>
	    ...
	    <plugin name='husky' filename='NRPGazeboGrpcJointControllerPlugin.so'>
	        <back_left_joint   P='10' I='0' D='0' Type='velocity' Target='0' IMax='0' IMin='0'/>
	        <back_right_joint  P='10' I='0' D='0' Type='velocity' Target='0' IMax='0' IMin='0'/>
	        <front_left_joint  P='10' I='0' D='0' Type='velocity' Target='0' IMax='0' IMin='0'/>
	        <front_right_joint P='10' I='0' D='0' Type='velocity' Target='0' IMax='0' IMin='0'/>
	    </plugin>
	    ...
	</model>

