.. index:: pair: page; Preprocessing Functions
.. _doxid-preprocessing_function:

Preprocessing Functions
=======================

Preprocessing Functions are introduced as a mean to optimize on expensive computations on datapacks attached to a single engine. In some cases there might be the need for applying the same operations on a particular datapack in multiple TFs. An example of this might be to apply a filter to a datapack containing an image coming from a physics simulator. In order to allow to execute this operations just once and let other TFs to access the processed datapack data, :ref:`PreprocessingFunctions <doxid-class_preprocessing_function>` (PFs) are introduced.

They are similar to :ref:`Transceiver Functions <doxid-transceiver_function>` both in implementation and behavior. Both are Python functions, their input and output are DataPacks and they are linked to an specific engine. PFs are also executed if and only if its linked Engine is synchronized.

They show two main differences with respect to TFs:

* Their output datapacks are not sent to the corresponding Engines, they are kept in a local datapack cache and can be used as input in TFs

* PFs just can take input datapacks from the Engine they are linked to

The latter is necessary to guarantee that new datapacks retrieved from a particular Engine are always processed by its connected PFs. In this way PFs can be thought as simple filters that read and transform datapacks coming from a certain Engine and store the processed data in the local datapack cache.

To declare a function as :ref:`PreprocessingFunction <doxid-class_preprocessing_function>`, the decorator:

.. ref-code-block:: cpp

	@:ref:`PreprocessingFunction <doxid-class_preprocessing_function>`("engine_name")

must be prepended to its definition.

In order to use the datapacks returned by PFs in other TFs, a dedicated decorator is available and must be used:

.. ref-code-block:: cpp

	@:ref:`PreprocessedDataPack <doxid-class_preprocessed_data_pack>`(keyword, id)

The difference between this decorator and :ref:`EngineDataPack <doxid-class_engine_data_pack>` is that with the latter it is indicated that the datapack should be requested from its linked engine. While the use the :ref:`PreprocessedDataPack <doxid-class_preprocessed_data_pack>` decorator tells that the datapack can be directly taken from the local datapack cache.

Since the output of PFs is stored in the local cache and does not need to process on the Engine Server side, PFs can return any type of :ref:`DataPack <doxid-class_data_pack>` without restrictions.

There is a :ref:`DataPack <doxid-class_data_pack>` type particularly convenient to use as PF output: **JsonDataPack**. This type of datapack stores a JSON object, and thus any type of data can be attached to it. Below is an example taking a camera image from :ref:`Gazebo <doxid-gazebo_engine>` and returning the processed data as a JsonDataPack object.

.. ref-code-block:: cpp

	from nrp_core import *
	import numpy as np
	
	@:ref:`EngineDataPack <doxid-class_engine_data_pack>`(keyword='camera', id=:ref:`DataPackIdentifier <doxid-struct_data_pack_identifier>`('husky_camera::camera', 'gazebo'))
	@:ref:`PreprocessingFunction <doxid-class_preprocessing_function>`("gazebo")
	def transceiver_function(camera):
	
	    # Return an empty datapack, if there's no data in camera datapack
	    if camera.isEmpty():
	        return [ :ref:`DataPackInterface <doxid-class_data_pack_interface>`("processed", "gazebo") ]
	
	    # Convert image to grayscale
	    rgb_weights = [0.2989, 0.5870, 0.1140]
	    d = np.frombuffer(camera.data.imageData, np.uint8)
	    image_data = d.reshape((camera.data.imageHeight,camera.data.imageWidth,3))
	
	    # Save image in grayscale in a datapack and return it
	    datapack = :ref:`JsonDataPack <doxid-class_data_pack>`("processed", "gazebo")
	    datapack.data["image_height"] = camera.data.imageHeight
	    datapack.data["image_width" ] = camera.data.imageWidth
	    datapack.data["grayscale"   ] = np.dot(image_data[...,:3], rgb_weights).tolist()
	
	    return [ datapack ]

