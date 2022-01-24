.. index:: pair: page; Engine DataPacks
.. _doxid-datapacks:

Engine DataPacks
================

DataPacks are simple objects which wrap around arbitrary data structures, like JSON objects or protobuf messages. They provide the necessary abstract interface, which is understood by all components of NRP-Core, while still allowing to pass data in various formats.

A :ref:`DataPack <doxid-class_data_pack>` consists of two parts:

* :ref:`DataPack <doxid-class_data_pack>` ID: which allows to uniquely indentify the object

* :ref:`DataPack <doxid-class_data_pack>` data: this is the data stored by the :ref:`DataPack <doxid-class_data_pack>`, which can be in principle of any type

DataPacks are mainly used by :ref:`Transceiver functions <doxid-transceiver_function>` to relay data between engines. Each engine type is designed to accept only datapacks of a certain type and structure. To discover which datapacks can be processed by each engine, check out the engine's documentation :ref:`here <doxid-nrp_engines>`.



.. _doxid-datapacks_1datapacks_id:

DataPack ID
~~~~~~~~~~~

Every datapack contains a :ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>`, which uniquely identifies the datapack object and allows for routing of the data between transceiver functions, engine clients and engine servers. A datapack identifier consists of three fields:

* name - name of the datapack. Must be unique.

* type - string representation of the :ref:`DataPack <doxid-class_data_pack>` data type. This field will most probably will be of no concern for the users. It is set and used internally and is not in human-readable form.

* engine name - name of the engine to which the datapack is bound.

These fields can be accessed from the transceiver functions:

.. ref-code-block:: cpp

	print(datapack.name)
	print(datapack.type)
	print(datapack.engine_name)





.. _doxid-datapacks_1datapacks_data:

DataPack data
~~~~~~~~~~~~~

:ref:`DataPack <doxid-class_data_pack>` is a template class with a single template parameter, which specifies the type of the data contained by the :ref:`DataPack <doxid-class_data_pack>`. This :ref:`DataPack <doxid-class_data_pack>` data can be in principle of any type. In practice there are some limitations though, since DataPacks, which are C++ objects, must be accessible from TransceiverFunctions, which are written in Python. Therefore the only :ref:`DataPack <doxid-class_data_pack>` data types which can be actually used in NRP-core are those for which Python bindings are provided. These are commented :ref:`below <doxid-datapacks_1supported_datapack_types>`.

In TransceiverFunctions, the :ref:`DataPack <doxid-class_data_pack>` data can always be accessed using the datapack "data" attribute.





.. _doxid-datapacks_1empty_datapack:

Empty DataPacks
~~~~~~~~~~~~~~~

It is possible for a datapack to contain no data. This is useful for example when an Engine is asked for a certain :ref:`DataPack <doxid-class_data_pack>` but it is not able to provide it. In this case, an Engine can return an empty datapack. This type of datapack contains only a datapack identifier and no data.

Attempting to retrieve the data from an empty :ref:`DataPack <doxid-class_data_pack>` will result in an exception. A method "isEmpty" is provided to check whether a :ref:`DataPack <doxid-class_data_pack>` is empty or not before attempting to access its data:

.. ref-code-block:: cpp

	if(not datapack.isEmpty()):
	    # It's safe to get the data
	    print(datapack.data)
	else:
	    # This will raise an exception
	    print(datapack.data)





.. _doxid-datapacks_1datapacks_tfs:

Role of DataPacks in TransceiverFunctions
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

DataPacks are both the input and output of TransceiverFunctions. When a datapack is declared as input of a TF, this datapack is always requested to the corresponding Engine when the latter is synchronized. When a datapack is returned by a TF, it is sent to the corresponding Engine after the TF is executed. For more information about the synchronization model used in NRP-core the reader can refer to these sections:

* :ref:`Transceiver Functions synchronization <doxid-transceiver_function_1transceiver_function_synchronization>`

* :ref:`NRP-core synchronization model <doxid-sync_model_details>`

The subsections below elaborate on the details of how to use DataPacks in TFs.



.. _doxid-datapacks_1datapacks_tfs_input:

DataPacks as input to transceiver functions
-------------------------------------------

DataPacks can be declared as :ref:`TransceiverFunction <doxid-class_transceiver_function>` inputs using the dedicated decorator. After that they can be accessed in TFs as input arguments.

.. ref-code-block:: cpp

	# Declare datapack with "datapack_name" name from engine "engine_name" as input using the @EngineDataPack decorator
	# The trasceiver function must accept an argument with the same name as "keyword" in the datapack decorator
	
	@:ref:`EngineDataPack <doxid-class_engine_data_pack>`(keyword="datapack", id=:ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>`("datapack_name", "engine_name"))
	@:ref:`TransceiverFunction <doxid-class_transceiver_function>`("engine_name")
	def transceiver_function(datapack):
	    print(datapack.data)
	
	# Multiple input datapacks from different engines can be declared
	
	@:ref:`EngineDataPack <doxid-class_engine_data_pack>`(keyword="datapack1", id=:ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>`("datapack_name1", "engine_name1"))
	@:ref:`EngineDataPack <doxid-class_engine_data_pack>`(keyword="datapack2", id=:ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>`("datapack_name2", "engine_name2"))
	@:ref:`TransceiverFunction <doxid-class_transceiver_function>`("engine_name1")
	def transceiver_function(datapack1, datapack2):
	    print(datapack1.data)
	    print(datapack2.data)

When passed as TF arguments, DataPacks behave at all effect as read-only objects. Even though it is possible to modify their data or to add new attributes to them inside of a :ref:`TransceiverFunction <doxid-class_transceiver_function>`, these changes will have no effect outside of the :ref:`TransceiverFunction <doxid-class_transceiver_function>`.





.. _doxid-datapacks_1datapacks_tfs_output:

DataPacks as output of transceiver functions
--------------------------------------------

DataPacks can be returned from the transceiver function.

.. ref-code-block:: cpp

	# NRP-Core expects transceiver functions to always return a list of datapacks
	
	@:ref:`TransceiverFunction <doxid-class_transceiver_function>`("engine_name")
	def transceiver_function():
	    datapack = :ref:`JsonDataPack <doxid-class_data_pack>`("datapack_name", "engine_name")
	
	    return [ datapack ]
	
	# Multiple datapacks can be returned
	
	@:ref:`TransceiverFunction <doxid-class_transceiver_function>`("engine_name")
	def transceiver_function():
	    datapack1 = :ref:`JsonDataPack <doxid-class_data_pack>`("datapack_name1", "engine_name")
	    datapack2 = :ref:`JsonDataPack <doxid-class_data_pack>`("datapack_name2", "engine_name")
	
	    return [ datapack1, datapack2 ]







.. _doxid-datapacks_1supported_datapack_types:

Supported DataPack data types
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As commented in the section above, DataPacks are both the input and output of TFs. Therefore, a conversion mechanism between C++ and Python is required for each supported :ref:`DataPack <doxid-class_data_pack>` data type. The types currently supported are nlohmann::json and protobuf messages. The subsections below give details of the Python API provided for each of these types.



.. _doxid-datapacks_1datapacks_json:

JsonDataPack
------------

**JsonDataPack** type wraps around `nlohmann::json <https://github.com/nlohmann/json>`__ C++ objects. The Python class wrapping the C++ json object is *NlohmannJson*, which is stored in JsonDataPack *data* attribute. **NlohmannJson** is very flexible and allows to pass most types of data between engines and transceiver functions without writing any additional code, it can contain all basic Python types. **NlohmannJson** also has partial support for numpy arrays - it's possible to use 1-dimensional arrays of integers and floats.



.. _doxid-datapacks_1datapacks_json_importing:

Importing and creating JsonDataPack
+++++++++++++++++++++++++++++++++++

To import JsonDataPack:

.. ref-code-block:: cpp

	from nrp_core.data.nrp_json import JsonDataPack

To create a JsonDataPack object:

.. ref-code-block:: cpp

	datapack = :ref:`JsonDataPack <doxid-class_data_pack>`("datapack_name", "engine_name")





.. _doxid-datapacks_1datapacks_json_setting_getting:

Getting and setting data
++++++++++++++++++++++++

Inside transceiver functions the data can be accessed like a python dictionary:

.. ref-code-block:: cpp

	# To set the data
	
	datapack = :ref:`JsonDataPack <doxid-class_data_pack>`("datapack_name", "engine_name")
	
	datapack.data["null"]   = None
	datapack.data["long"]   = 1
	datapack.data["double"] = 43.21
	datapack.data["string"] = "string"
	datapack.data["bool"]   = True
	datapack.data["array"]  = [5, 1, 6]
	datapack.data["tuple"]  = (1, 2, 3)
	datapack.data["object"] = {"key1": "value", "key2": 600}
	
	# To retrieve the data
	
	print(datapack.data["string"])
	print(datapack.data["object"])
	
	# JsonDataPack supports dict's __getitem__ and keys methods.
	for k in datapack.data.keys():
	    print(datapack.data[k])

Numpy arrays:

.. ref-code-block:: cpp

	# Numpy integer arrays
	
	datapack.data["numpy_array_int8" ] = np.array([ 1,  2,  3], dtype="int8")
	datapack.data["numpy_array_int16"] = np.array([ 4,  5,  6], dtype="int16")
	datapack.data["numpy_array_int32"] = np.array([ 7,  8,  9], dtype="int32")
	datapack.data["numpy_array_int64"] = np.array([10, 11, 12], dtype="int64")
	
	# Numpy unsigned integer arrays
	
	datapack.data["numpy_array_uint8" ] = np.array([0,  1,  2], dtype="uint8")
	datapack.data["numpy_array_uint16"] = np.array([3,  4,  5], dtype="uint16")
	datapack.data["numpy_array_uint32"] = np.array([6,  7,  8], dtype="uint32")
	datapack.data["numpy_array_uint64"] = np.array([9, 10, 11], dtype="uint64")
	
	# Numpy floating-point arrays
	
	datapack.data["numpy_array_float32"] = np.array([0.5, 2.3, 3.55], dtype="float32")
	datapack.data["numpy_array_float64"] = np.array([1.5, 2.3, 3.88], dtype="float64")





.. _doxid-datapacks_1datapacks_json_inspecting:

Inspecting content of JsonDataPack
++++++++++++++++++++++++++++++++++

Printing the content using Python's built-in function **str** :

.. ref-code-block:: cpp

	str(datapack.data)
	str(datapack.data["array"])
	str(datapack.data["object"])
	
	# Or print it directly:
	
	print(datapack.data)

Getting a list of keys:

.. ref-code-block:: cpp

	keys = datapack.data.keys()

Getting length of the object:

.. ref-code-block:: cpp

	length = len(datapack.data)

The above will return number of keys, if data is a JSON object, or number of elements, if it's a JSON array.





.. _doxid-datapacks_1datapacks_json_arrays:

Using JsonDataPacks to store JSON arrays
++++++++++++++++++++++++++++++++++++++++

In all the examples above it has been assumed that JsonDataPack is storing a JSON object. Actually the data object can contain either a JSON object, a JSON array or an empty object, it depends on how it is started. After instantiation, it contains an empty object:

.. ref-code-block:: cpp

	datapack = :ref:`JsonDataPack <doxid-class_data_pack>`("example_datapack", "example_engine")
	# Returned value is 'null'
	datapack.data.json_type()

If data is appended to it, the datapack stores a JSON array:

.. ref-code-block:: cpp

	datapack.data.append(1.55)
	# Returned value is 'array'
	datapack.data.json_type()

If instead a key is assigned, it stores a JSON object:

.. ref-code-block:: cpp

	datapack = :ref:`JsonDataPack <doxid-class_data_pack>`("example_datapack", "example_engine")
	datapack.data['key'] = 1.55
	# Returned value is 'object'
	datapack.data.json_type()

Please be aware that methods specific to 'object' type for accessing or setting elements will raise an error when used with an 'array' type and otherwise.







.. _doxid-datapacks_1datapacks_protobuf:

Protobuf datapacks
------------------

In contrast with JsonDataPack, which can wrap any nlohmann::json C++ object, a Python wrapper class is generated for each Protobuf definition. For example, for the *Camera* message listed below (which is used by the :ref:`Gazebo Engine <doxid-gazebo_engine>`), a python class *GazeboCameraDataPack* is generated.

.. ref-code-block:: cpp

	package Gazebo;
	
	// Data coming from gazebo camera datapack
	message Camera
	{
	    uint32 imageWidth  = 1;
	    uint32 imageHeight = 2;
	    uint32 imageDepth  = 3;
	    bytes  imageData   = 4;
	}

This class contains a *data* attribute which is of type *GazeboCamera* and gives access to the wrapped datapack data. The generated Python classes match the original Protobuf Python API as described in the `protobuf documentation <https://developers.google.com/protocol-buffers/docs/reference/python-generated>`__. There are some known limitations with respect to the original Protobuf Python API which are listed below with references to the protobuf documentation:

#. Well Known Types not supported `ref <https://developers.google.com/protocol-buffers/docs/reference/python-generated#wkt>`__

#. Repeated Message field not supported `ref <https://developers.google.com/protocol-buffers/docs/reference/python-generated#repeated-message-fields>`__

#. Map field type not supported `ref <https://developers.google.com/protocol-buffers/docs/reference/python-generated#map-fields>`__

#. Only basic Enum support. To set or get *Enum* fields only *int* can be used. *Enum constants* can't be accessed from python `ref <https://developers.google.com/protocol-buffers/docs/reference/python-generated#enum>`__

#. The *Message* Python wrapper only supports a subset of the methods listed `here <https://googleapis.dev/python/protobuf/latest/google/protobuf/message.html>`__. These are: 'Clear', 'ClearField', 'HasField', 'IsInitialized' and 'WhichOneof'.

Finally, these Python wrappers are automatically generated in the NRP-core build process. See this :ref:`guide <doxid-tutorial_add_proto_definition>` to know how to add custom message definitions so they become afterwards available to Engines and TFs.





.. _doxid-datapacks_1datapacks_rosmsg:

ROS msg datapacks
-----------------

Similarly to Protobuf datapacks, a Python wrapper class is generated for each ROS msg definition. For example, for a message of type ``Pose`` from package ``geometry_msgs``, a python class *PoseDataPack* is generated. This class contains a *data* attribute which is of type *Pose* and gives access to the wrapped datapack data. The generated Python bindings can be found under the Python module ``nrp_core.data.nrp_ros``. For example, in the case of the ``Pose`` message:

.. ref-code-block:: cpp

	from nrp_core.data.nrp_ros.geometry_msgs import PoseDataPack
	p = PoseDataPack("name","engine")
	type(p.data) # output is <class 'nrp_core.data.nrp_ros.geometry_msgs.Pose'>

By default Python bindings are generated for all message types in the next ROS packages:

* nrp_ros_msgs (package containing NRP-core ROS message definitions)

* std_msgs

* geometry_msgs

* sensor_msgs

Any message definition contained in these packages can be used in TransceiverFunctions directly. It is also possible to generate Python bindings for messages in other ROS packages by adding them to the variable ``NRP_CATKIN_PACKAGES`` defined in the root ``CMakeLists.txt`` file. Also, this guide: :ref:`Adding new ROS message definitions <doxid-tutorial_add_ros_msg_definition>`, explains how to extend ``nrp_ros_msgs`` with new message definitions.

The generated binding classes match the original ROS Python API with only one exception. In ROS Python, fields of type ``array`` are stored in objects of type ``list`` while in the case of the Python binding generated by NRP-core a new class is created for each ``array field``. To illustrate the former, consider the case of ``UInt64MultiArray`` message defined in the ``std_msgs`` package:

.. ref-code-block:: cpp

	# msg definition in std_msgs package
	MultiArrayLayout  layout        # specification of data layout
	uint64[]          data          # array of data

In ROS Python the field ``data`` is stored in a ``list``, while in the NRP-core Python wrapper class it is stored in a new Python class of type ``UInt64MultiArray_data`` :

.. ref-code-block:: cpp

	import nrp_core.data.nrp_ros.std_msgs as nrp_std
	a = nrp_std.UInt64MultiArray()
	type(a.data) # output is <class 'nrp_core.data.nrp_ros.std_msgs.UInt64MultiArray_data'>

This new class behaves similarly to a Python ``list``. It supports ``append`` and ``extend`` methods for adding new data. Its elements can accessed by index or iterated and ``len`` will return the number of elements.







.. _doxid-datapacks_1datapacks_implementation:

Implementation details
~~~~~~~~~~~~~~~~~~~~~~

All concrete datapack classes should be based on the :ref:`DataPack <doxid-class_data_pack>` class. It is a template class, and the single template argument specifies the data structure type, which is going to be held by the class instances.

The :ref:`DataPack <doxid-class_data_pack>` class design is somewhat similar to that of std::unique_ptr. Whenever a datapack object is constructed, it takes ownership of the input data structure. This structure may be then accessed and modified, or the ownership may be released.

The :ref:`DataPack <doxid-class_data_pack>` class inherits from :ref:`DataPackInterface <doxid-class_data_pack_interface>`. This class may also be instantiated, but the object will not carry any data (ie. it's an empty :ref:`DataPack <doxid-class_data_pack>`).



.. _doxid-datapacks_1datapacks_implementation_empty:

Empty datapacks
---------------

A :ref:`DataPack <doxid-class_data_pack>` class is considered empty when its data is released. Every instance of the base class, :ref:`DataPackInterface <doxid-class_data_pack_interface>`, is also considered empty, because there is no data stored in it.





.. _doxid-datapacks_1datapacks_implementation_python:

Python interface
----------------

In order to be accessible to transceiver functions, a conversion mechanism between C++ and Python must be specified for each :ref:`DataPack <doxid-class_data_pack>` data type. Currently NRP-core provides Python bindings for nlohmann::json and protobuf messages. In case you wished to integrate a different data type, you would have to implement Python bindings for this type and make them available to NRP-core as a Python module.

