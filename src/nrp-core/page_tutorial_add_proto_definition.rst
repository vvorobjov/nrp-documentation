.. index:: pair: page; Adding new protobuf message definitions
.. _doxid-tutorial_add_proto_definition:

Adding new protobuf message definitions
=======================================

This guide shows how to extend the protobuf message definitions availabe in NRP-core. Afterwards, the new definitions become available to use in TransceiverFunctions and gRPC Engines.

The process is very simple and consists only of two steps:

#. Add your new message definitions into a separate \*.proto\* file. Refer to `proto3 language guide <https://developers.google.com/protocol-buffers/docs/proto3>`__ to check available possibilities.

#. Place the new \*.proto\* file in the folder *nrp_protobuf/engine_proto_defs*. In this way your message defitinions will be automatically compiled during the NRP-core build process. Also, the Python bindings required (see :ref:`Protobuf datapacks <doxid-datapacks_1datapacks_protobuf>` section) to use these new proto classes in NRP-core will be generated and compiled.

Optionally, those new message definitions which will be used to exchange data with gRPC Engines must be manually added to the *Engine.DataPackMessage* message oneof *data* field definition. Engine.DataPackMessage\* is the message type used by gRPC Engine servers to send data to gRPC Engine clients. It contains a :ref:`DataPack <doxid-class_data_pack>` Id and the data itself, stored in a oneof *data* field. *Oneof* is a kind of XOR type of field (please refer to the guide listed above for more information). So, in the end, only those message types listed in the oneof *data* field can be sent by gRPC Engine servers.

The *Engine.DataPackMessage* can be found in the folder *nrp_protobuf/nrp_proto_defs*. As shipped with NRP-core, it contains only a message definition used for testing and those messages used by the GazeboGrpcEngine.

.. ref-code-block:: cpp

	message DataPackMessage
	{
	    DataPackIdentifier datapackId = 1;
	
	    oneof data
	    {
	        EngineTest.TestPayload  test   = 2;
	        Gazebo.Camera camera = 3;
	        Gazebo.Link   link   = 4;
	        Gazebo.Joint  joint  = 5;
	        Dump.String   dump_string  = 6;
	        Dump.ArrayFloat   dump_float  = 7;
	    }
	}

