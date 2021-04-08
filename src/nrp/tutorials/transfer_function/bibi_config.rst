Tutorial: Writing a BIBI Configuration
======================================

.. todo:: Add author/responsible
.. todo:: Consider duplication :doc:`/nrp/modules/CLE/hbp_nrp_cle/tutorials/bibi_config`, https://hbpneurorobotics.atlassian.net/l/c/iHd8of31

A BIBI Configuration contains all the information necessary to couple a brain model with a robot model using transfer functions and to run these simulations
in the Closed Loop Engine (CLE). Thus, besides references to the brain model and the robot model, it contains the specifications of the TFs.

As an XML file, such a specification may be created by tools. We have an XML Schema document to validate BIBI Configuration files.

As the complete metamodel of the BIBI Configuration may be a bit complicated at the beginning, we build a BIBI Configuration stepwise in XML.
However, at any point in time, you may find it useful to lookup :doc:`../../specifications/bibi_configuration` for a complete reference.

To begin, we start with a new BIBI Configuration. BIBI configuration files should have the file extension **.bibi**.

.. code-block:: xml

    <bibi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
          xmlns="http://schemas.humanbrainproject.eu/SP10/2014/BIBI" >
    </bibi>

Here we have simply created a *bibi* element in the BIBI Configuration namespace.

.. note:: 
    The namespace is subject to change. In particular, the XML Schema definition currently cannot be obtained via the given URL. In order to allow editors to provide tool support,
    we recommend to explicitly specify the location of the schema manually. This can be done using the following XML attribute (assuming that the schema file resides in *../bibi_configuration.xsd*):
    *xsi:schemaLocation="http://schemas.humanbrainproject.eu/SP10/2014/BIBI ../bibi_configuration.xsd*

Specification of the neuronal network
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

At the moment our BIBI Configuration is not valid. One of the reasons for this is that it lacks a description for the used neuronal network. We need to tell the CLE
what brain models we are going to use and what neuron groups exist.

.. code-block:: xml

    <brainModel>
      <file>brain_model/braitenberg.py</file>
      <populations population="sensors" xsi:type="Range" from="0" to="5"/>
      <populations population="actors" xsi:type="Range" from="6" to="8"/>
    </brainModel>

This code block must be inserted as a child element of the root *bibi* element.

In this example, we have specified that the brain model from *braitenberg.py* should be used. This path is either absolute or
relative to the **NRP_MODELS_DIRECTORY** environment variable. Models of neural networks are usually in the *brain_models* directory inside this directory.

.. note::
    When using the CLE through the :abbr:`NRP (Neurorobotics Platform)` platform, the **NRP_MODELS_DIRECTORY** will be your user directory.
    When using the CLE separately as e.g. for development machines, this environment variable should be set to a Models repository clone.

We further specify two neuron groups, the sensors ranging from 0 to 5 (exclusive) and the actors from 6 to and excluding 8. Note that these neuron groups exactly match the
neuron groups (colors) from :doc:`setup page<setup>`. We will use these neuron groups in the TFs for reference.

.. note:: The XML Schema enforces that the neuronal network file has the correct extension **.py** (or *.h5* for legacy h5 formatted network files).

Specification of the robot model
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Our BIBI Configuration also needs a robot model. We specify it simply by the file name of the robot model in the SDF format. This file then contains both the robot meshes as well as
information on the plugins used by this robot.

.. code-block:: xml

    <bodyModel>husky_model/model.sdf</bodyModel>

.. note:: The XML Schema enforces that the brain model has the correct file extension **.sdf**.

Up to this point, the BIBI Configuration should look as follows:

.. code-block:: xml

    <?xml version="1.0" encoding="UTF-8"?>
    <bibi xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
          xmlns="http://schemas.humanbrainproject.eu/SP10/2014/BIBI">
      <brainModel>
        <file>brain_model/braitenberg.py</file>
        <populations population="sensors" xsi:type="Range" from="0" to="5"/>
        <populations population="actors" xsi:type="Range" from="6" to="8"/>
      </brainModel>
      <bodyModel>husky_model/model.sdf</bodyModel>
    </bibi>

While we now have created a valid BIBI Configuration, it does not yet contain any TF, so the simulations will run in parallel with no connection to each other.
To learn how to specify TFs, see :doc:`neuron2robot`.

Transfer Functions
^^^^^^^^^^^^^^^^^^

There are three ways to include a Transfer Function into a BIBI model: 

- Reference an existing Transfer function from a Python file
- Include the Python code directly in the BIBI model
- Include a model-based description of the Transfer Function in the BIBI model

To reference a transfer function from a file, the following code in the BIBI model is sufficient:

.. code-block:: xml

  <transferFunction xsi:type="PythonTransferFunction" src="your_tf.py"/>

The file extension of the specified file does not matter as the file is loaded line by line and then executed in a sandboxed environment through RestrictedPython.

Alternatively, the Python code for the Transfer Function may also be specified in-place:

.. code-block:: xml

  <transferFunction xsi:type="PythonTransferFunction">
    #<![CDATA[
    @nrp.MapRobotSubscriber("camera", Topic('/husky/camera', sensor_msgs.msg.Image))
    @nrp.MapSpikeSource("red_left_eye", nrp.brain.sensors[slice(0, 3, 2)], nrp.poisson)
    @nrp.MapSpikeSource("red_right_eye", nrp.brain.sensors[slice(1, 4, 2)], nrp.poisson)
    @nrp.MapSpikeSource("green_blue_eye", nrp.brain.sensors[4], nrp.poisson)
    @nrp.Robot2Neuron()
    def eye_sensor_transmit(t, camera, red_left_eye, red_right_eye, green_blue_eye):
        """
        This transfer function uses OpenCV to compute the percentages of red pixels
        seen by the robot on his left and on his right. Then, it maps these percentages
        (see decorators) to the neural network neurons using a Poisson generator.
        """
        bridge = CvBridge()
        red_left = red_right = green_blue = 0.0
        if not isinstance(camera.value, type(None)):

            # Boundary limits of what we consider red (in HSV format)
            lower_red = np.array([0, 30, 30])
            upper_red = np.array([0, 255, 255])

            # Get an OpenCV image
            cv_image = bridge.imgmsg_to_cv2(camera.value, "rgb8")

            # Transform image to HSV (easier to detect colors).
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2HSV)

            # Create a mask where every non red pixel will be a zero.
            mask = cv2.inRange(hsv_image, lower_red, upper_red)
            image_size = (cv_image.shape[0] * cv_image.shape[1])

            if (image_size > 0):
                # Since we want to get left and right red values, we cut the image
                # in 2.
                half = cv_image.shape[1] // 2

                # Get the number of red pixels in the image.
                red_left = cv2.countNonZero(mask[:, :half])
                red_right = cv2.countNonZero(mask[:, half:])

                # We have to multiply the red rates by 2 since it is for an
                # half image only. We also multiply all of them by 1000 so that
                # we have enough spikes produced by the Poisson generator
                red_left_eye.rate = 2 * 1000 * (red_left / float(image_size))
                red_right_eye.rate = 2 * 1000 * (red_right / float(image_size))
                green_blue_eye.rate = 75 * ((image_size - (red_left + red_right)) / float(image_size))
    #]]>
  </transferFunction>

In the third option, we also allow to specify a Transfer Function in the BIBI model directly. 
This way is designed for tool interactivity, not for a manual specification.
The following Transfer Function makes use of a built-in function to detect red pixels and is specified entirely in the BIBI model:

.. code-block:: xml

    <device name="left_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="1"/>
    </device>
    <device name="right_wheel_neuron" type="LeakyIntegratorAlpha">
      <neurons xsi:type="Index" population="actors" index="2"/>
    </device>
    <returnValue name="wheel" topic="/husky/cmd_vel" type="geometry_msgs.msg.Twist">
      <body xsi:type="Call" type="geometry_msgs.msg.Twist">
        <argument name="linear">
          <value xsi:type="Call" type="geometry_msgs.msg.Vector3">
            <argument name="x">
              <value xsi:type="Scale" factor="20">
                <inner xsi:type="Min">
                  <operand xsi:type="ArgumentReference" name="left_wheel_neuron" property="voltage"/>
                  <operand xsi:type="ArgumentReference" name="right_wheel_neuron" property="voltage"/>
                </inner>
              </value>
            </argument>
            <argument name="y">
              <value xsi:type="Constant" value="0"/>
            </argument>
            <argument name="z">
              <value xsi:type="Constant" value="0"/>
            </argument>
          </value>
        </argument>
        <argument name="angular">
          <value xsi:type="Call" type="geometry_msgs.msg.Vector3">
            <argument name="x">
              <value xsi:type="Constant" value="0"/>
            </argument>
            <argument name="y">
              <value xsi:type="Constant" value="0"/>
            </argument>
            <argument name="z">
              <value xsi:type="Scale" factor="100">
                <inner xsi:type="Subtract">
                  <operand xsi:type="ArgumentReference" name="right_wheel_neuron" property="voltage"/>
                  <operand xsi:type="ArgumentReference" name="left_wheel_neuron" property="voltage"/>
                </inner>
              </value>
            </argument>
          </value>
        </argument>
      </body>
    </returnValue>
  </transferFunction>
  <transferFunction xsi:type="Robot2Neuron" name="eye_sensor_transmit">
    <local name="image_results">
      <body xsi:type="Call" type="hbp_nrp_cle.tf_framework.tf_lib.detect_red">
        <argument name="image">
          <value xsi:type="ArgumentReference" name="camera" property="value"/>
        </argument>
      </body>
    </local>
    <device name="red_left_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="0" to="3" step="2"/>
      <!--body xsi:type="Scale" factor="0.002"-->
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="left"/>
      </body>
    </device>
    <device name="red_right_eye" type="Poisson">
      <neurons xsi:type="Range" population="sensors" from="1" to="4" step="2"/>
      <!--body xsi:type="Scale" factor="0.002"-->
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="right"/>
      </body>
    </device>
    <device name="green_blue_eye" type="Poisson">
      <neurons xsi:type="Index" population="sensors" index="4"/>
      <!--body xsi:type="Scale" factor="0.00025"-->
      <body xsi:type="Scale" factor="1000.0">
        <inner xsi:type="ArgumentReference" name="image_results" property="go_on"/>
      </body>
    </device>
    <topic name="camera" topic="/husky/camera" type="sensor_msgs.msg.Image"/>
  </transferFunction>
