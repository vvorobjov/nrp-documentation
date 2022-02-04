.. index:: pair: page; Gazebo DataPacks
.. _doxid-gazebo_datapacks:

Gazebo DataPacks
================

The JSON implementation always uses :ref:`JsonDataPack <doxid-datapacks_1datapacks_json>` to exchange data with TFs. This datapack can contain arbitrary data in a JSON object. The gRPC implementation uses :ref:`Protobuf datapacks <doxid-datapacks_1datapacks_protobuf>`. Below are listed the protobuf message definitions used by the Gazebo engine:

.. ref-code-block:: cpp

	syntax = "proto3";
	
	package Gazebo;
	
	/*
	 * Data coming from gazebo camera datapack
	 */
	message Camera
	{
	    uint32 imageWidth  = 1;
	    uint32 imageHeight = 2;
	    uint32 imageDepth  = 3;
	    bytes  imageData   = 4;
	}
	
	/*
	 * Data coming from gazebo link datapack
	 */
	message Link
	{
	    repeated float position        = 1;
	    repeated float rotation        = 2;
	    repeated float linearVelocity  = 3;
	    repeated float angularVelocity = 4;
	}
	
	/*
	 * Data coming from gazebo joint datapack
	 */
	message Joint
	{
	    float position = 1;
	    float velocity = 2;
	    float effort   = 3;
	}
	
	// EOF

For each of these definitions, a datapack Python class is generated, see :ref:`here <doxid-datapacks_1datapacks_protobuf>` for more details. Concretely, these datapack types are generated:

* GazeboCameraDataPack: contains a camera image

* GazeboJointDataPack: contains a single joint state information

* GazeboLinkDataPack: contains a single link state information

In the case of the JSON implementation, even if :ref:`JsonDataPack <doxid-datapacks_1datapacks_json>`, the content of the datapacks data will be the same, but stored as a JSON object. This content is summarized below for each datapack type.

The GazeboCameraDataPack consists of the following attributes: ============  =============================================  ==================================================================  ==========================  
Attribute     Description                                    Python Type                                                         C type                      
============  =============================================  ==================================================================  ==========================  
image_height  Camera Image height                            uint32                                                              uint32                      
image_width   Camera Image width                             uint32                                                              uint32                      
image_depth   Camera Image depth. Number of bytes per pixel  uint8                                                               uint32                      
image_data    Camera Image data. 1-D array of pixel data     numpy.array(image_height * image_width * image_depth, numpy.uint8)  std::vector<unsigned char>  
============  =============================================  ==================================================================  ==========================

The GazeboJointDataPack consists of the following attributes: =========  ===============================  ===========  ======  
Attribute  Description                      Python Type  C type  
=========  ===============================  ===========  ======  
position   Joint angle position (in rad)    float        float   
velocity   Joint angle velocity (in rad/s)  float        float   
effort     Joint angle effort (in N)        float        float   
=========  ===============================  ===========  ======

The GazeboLinkDataPack consists of the following attributes: =========  ===========================  =============================  ===================  
Attribute  Description                  Python Type                    C type               
=========  ===========================  =============================  ===================  
pos        Link Position                numpy.array(3, numpy.float32)  std::array<float,3>  
rot        Link Rotation as quaternion  numpy.array(4, numpy.float32)  std::array<float,4>  
lin_vel    Link Linear Velocity         numpy.array(3, numpy.float32)  std::array<float,3>  
ang_vel    Link Angular Velocity        numpy.array(3, numpy.float32)  std::array<float,3>  
=========  ===========================  =============================  ===================

Each of this attributes can be accessed under their respective names from the *data* attribute of each :ref:`DataPack <doxid-class_data_pack>` type.

