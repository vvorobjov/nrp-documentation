Experiment Model Specification
==============================

.. todo:: Add author/responsible

Schema Document Properties
--------------------------
:Target Namespace: http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig
:Element Qualifiers: By default, local element declarations belong to this schema's target namespace.
:Attribute Qualifiers: By default, local attribute declarations have no namespace.
:Imports: This schema imports schema(s) from the following namespace(s):
  - http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml


Declared Namespaces
-------------------

+-------------------+---------------------------------------------------------------------------------------------------------+
| Prefix            | Namespace                                                                                               |
+===================+=========================================================================================================+
| (default)         | http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig                                                 |
+-------------------+---------------------------------------------------------------------------------------------------------+
| xml               | http://www.w3.org/XML/1998/namespace                                                                    |
+-------------------+---------------------------------------------------------------------------------------------------------+
| xsd               | http://www.w3.org/2001/XMLSchema                                                                        |
+-------------------+---------------------------------------------------------------------------------------------------------+
| xsi               | http://www.w3.org/2001/XMLSchema-instance                                                               |
+-------------------+---------------------------------------------------------------------------------------------------------+
| bibi              | http://schemas.humanbrainproject.eu/SP10/2014/BIBI                                                      |
+-------------------+---------------------------------------------------------------------------------------------------------+
| sc                | http://schemas.humanbrainproject.eu/SP10/2015/ExDConfig/scxml                                           |
+-------------------+---------------------------------------------------------------------------------------------------------+
| tns               | http://schemas.humanbrainproject.eu/SP10/2014/ExDConfig                                                 |
+-------------------+---------------------------------------------------------------------------------------------------------+



Global Declarations
-------------------
   
Element: ExD
^^^^^^^^^^^^

:Name: ExD
:Type: ExD
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The root element of a experiment configuration model must be an ExD object.

XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    
       <ExD> ExD </ExD>



Global Definitions
------------------
   
Complex Type: BibiConf
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: BibiConf
:Abstract: no
:Documentation: 
  This type denotes the BIBI configuration used for this experiment. It is described using a reference to the BIBI model in the src attribute and an attribute processes to specify the number of processes that should be used to run the experiment. The default value for processes is 1.

Properties
""""""""""

    
    
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [1]
:Documentation: 
  The path to the BIBI configuration that specifies the model, the neural network and the connection between those.

    
Attribute processes
~~~~~~~~~~~~~~~~~~~
:Name: processes
:Type: positiveInteger
:Multiplicity: [0..1]
:Default Value: 1
:Documentation: 
  The number of processes that should be used to run the neural network simulation. If this value is larger than 1, a dedicated simulation setup for distributed simulation of the neural network is used.

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [1]" processes="positiveInteger [0..1]"/> 


Complex Type: CameraPose
^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: CameraPose
:Abstract: no
:Documentation: 
  This type denotes a camera pose. Unlike the robot pose, a camera pose is specified using a position of the camera and a point to which the camera looks at. The camera is always rotated with the up vector z (0,0,1).

Properties
""""""""""

    
    
      
Element cameraPosition
~~~~~~~~~~~~~~~~~~~~~~
:Name: cameraPosition
:Type: Position
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The position of the camera

      
Element cameraLookAt
~~~~~~~~~~~~~~~~~~~~
:Name: cameraLookAt
:Type: Position
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The position to which the camera should look at

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <cameraPosition> Position </cameraPosition> [1]
       <cameraLookAt> Position </cameraLookAt> [1]
    </...>


Complex Type: ConfFile
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ConfFile
:Abstract: no
:Documentation: 
  This type denotes a configuration entry. Configuration entries are used for multiple purposes, therefore the type of the configuration entry is set explicitly in an attribute called type. The actual configuration is referenced as a file through the src attribute.

Properties
""""""""""

    
    
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [1]
:Documentation: 
  The path to the file that acts as configuration. Files specified as configuration are automatically considered whe an experiment is deployed.

    
Attribute type
~~~~~~~~~~~~~~
:Name: type
:Type: ConfType
:Multiplicity: [1]
:Documentation: 
  The type of the configuration entry describes what this entry is used for. The NRP allows both predefined and custom entries.

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [1]" type="ConfType [1]"/> 


Complex Type: EnvironmentModel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: EnvironmentModel
:Abstract: no
:Documentation: 
  This type defines the necessary configuration for an environment. It combines the specification of an environment model through the src attribute and a robot pose using the element robotPose.

Properties
""""""""""

    
    
      
Element robotPose
~~~~~~~~~~~~~~~~~
:Name: robotPose
:Type: RobotPose
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The position of the robot

    
    
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [1]
:Documentation: 
  A path to an SDF file that specifies the scene

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [1]"> 
       <robotPose> RobotPose </robotPose> [1]
    </...>


Complex Type: ExD
^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ExD
:Abstract: no
:Documentation: 
  This type is the root type for an experiment configuration.

Properties
""""""""""

    
    
      
Element name
~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  This element denotes the name of the experiment as it appears in the experiment list.

      
Element thumbnail
~~~~~~~~~~~~~~~~~
:Name: thumbnail
:Type: ThumbnailFile
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  This element references a path to a thumbnail that is used to give the user a forecast to the experiment.

      
Element description
~~~~~~~~~~~~~~~~~~~
:Name: description
:Type: string
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  This description will appear in the experiment description and provide a short description explaining what the experiment is all about.

      
Element timeout
~~~~~~~~~~~~~~~
:Name: timeout
:Type: double
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The timeout of an experiment is the time an experiment is allowed to run by default, specified in seconds. If that time has elapsed, the users are asked whether they want to extend the runtime of the simulation. On the servers, this will only be allowed if the timeout fits within the cluster allocation.

      
Element configuration
~~~~~~~~~~~~~~~~~~~~~
:Name: configuration
:Type: ConfFile
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  An experiment may have multiple configuration entries. Despite configuration entries can be specified in anywhere in the ExD element, they must appear together.

      
Element maturity
~~~~~~~~~~~~~~~~
:Name: maturity
:Type: MaturityType
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The maturity of an experiment determines whether it is shown by default to the user or only browsable in dev mode.

      
Element environmentModel
~~~~~~~~~~~~~~~~~~~~~~~~
:Name: environmentModel
:Type: EnvironmentModel
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The environment model of an experiment specifies the used world file for a simulation and the pose where the robot should be spawned.

      
Element visualModel
~~~~~~~~~~~~~~~~~~~
:Name: visualModel
:Type: VisualModel
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  With the visual model, an experiment can specify an alternatively used model for the frontend visualization. This is helpful in case the robot model used in gazebo is very detailed and thus hard to visualize on the client. On the server, there may be more resources available to simulate more complex models.

      
Element bibiConf
~~~~~~~~~~~~~~~~
:Name: bibiConf
:Type: BibiConf
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The bibiConf element of an experiment configuration specifies the 

      
Element experimentControl
~~~~~~~~~~~~~~~~~~~~~~~~~
:Name: experimentControl
:Type: ExperimentControl
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The experiment control lists all state machines that control the experiment.

      
Element experimentEvaluation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~
:Name: experimentEvaluation
:Type: ExperimentControl
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The experiment evaluation element lists all state machines that evaluate the success of a simulated experiment.

      
Element cameraPose
~~~~~~~~~~~~~~~~~~
:Name: cameraPose
:Type: CameraPose
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The camera pose specifies the initial position of the camera when a simulation is started.

      
Element rosLaunch
~~~~~~~~~~~~~~~~~
:Name: rosLaunch
:Type: RosLaunch
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The roslaunch element species the path to a ROSLaunch file that is executed when the experiment is simulated. If no file is specified, no ROSLaunch file is executed at the beginning of an experiment.

      
Element rngSeed
~~~~~~~~~~~~~~~
:Name: rngSeed
:Type: positiveInteger
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  If specified, this element specifies the random number generator seed. If this field is left blank, a seed is generated and therefore, the simulation is not 100% deterministic. If a seed is specified here, this seed is used for the robot and neural simulation, making the simulation much more deterministic.

      
Element physicsEngine
~~~~~~~~~~~~~~~~~~~~~
:Name: physicsEngine
:Type: PhysicsEngine
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  If specified, this element denotes the physics simulator that should be used. We currently support either ODE or OpenSim.

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <name> string </name> [1]
       <thumbnail> ThumbnailFile </thumbnail> [1]
       <description> string </description> [1]
       <timeout> double </timeout> [0..1]
       <configuration> ConfFile </configuration> [0..*]
       <maturity> MaturityType </maturity> [0..1]
       <environmentModel> EnvironmentModel </environmentModel> [1]
       <visualModel> VisualModel </visualModel> [0..1]
       <bibiConf> BibiConf </bibiConf> [1]
       <experimentControl> ExperimentControl <!-- Uniqueness Constraint - uniqueExperimentControlSelector - tns:stateMachineField(s) - @id--></experimentControl> [0..1]
       <experimentEvaluation> ExperimentControl <!-- Uniqueness Constraint - uniqueExperimentEvaluationSelector - tns:stateMachineField(s) - @id--></experimentEvaluation> [0..1]
       <cameraPose> CameraPose </cameraPose> [0..1]
       <rosLaunch> RosLaunch </rosLaunch> [0..1]
       <rngSeed> positiveInteger </rngSeed> [0..1]
       <physicsEngine> PhysicsEngine </physicsEngine> [0..1]
    </...>


Complex Type: ExperimentControl
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ExperimentControl
:Abstract: no
:Documentation: 
  This type depicts a list of state machines

Properties
""""""""""

    
    
      
Element stateMachine
~~~~~~~~~~~~~~~~~~~~
:Name: stateMachine
:Type: StateMachine
:Multiplicity: [1..*]
:Nillable: no
:Abstract: no
:Documentation: 
  The actual state machines of this list of state machines

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <stateMachine> StateMachine </stateMachine> [1..*]
    </...>


Complex Type: Position
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Position
:Abstract: no
:Documentation: 
  This type denotes a position with x, y and z coordinates.

Properties
""""""""""

    
    
Attribute x
~~~~~~~~~~~
:Name: x
:Type: double
:Multiplicity: [1]
:Documentation: 
  The x coordinate of the position

    
Attribute y
~~~~~~~~~~~
:Name: y
:Type: double
:Multiplicity: [1]
:Documentation: 
  The y coordinate of the position

    
Attribute z
~~~~~~~~~~~
:Name: z
:Type: double
:Multiplicity: [1]
:Documentation: 
  The z coordinate of the position

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... x="double [1]" y="double [1]" z="double [1]"/> 


Complex Type: RobotPose
^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: RobotPose
:Abstract: no
:Documentation: 
  This type represents a robot pose. It consists of a position part (x, y and z coordinates) and a rotation part (roll, pitch and yaw). All fields are double precision values.

Properties
""""""""""

    
    
Attribute x
~~~~~~~~~~~
:Name: x
:Type: double
:Multiplicity: [1]
:Documentation: 
  The x coordinate of the robot position

    
Attribute y
~~~~~~~~~~~
:Name: y
:Type: double
:Multiplicity: [1]
:Documentation: 
  The y coordinate of the robot position

    
Attribute z
~~~~~~~~~~~
:Name: z
:Type: double
:Multiplicity: [1]
:Documentation: 
  The z coordinate of the robot position

    
Attribute roll
~~~~~~~~~~~~~~
:Name: roll
:Type: double
:Multiplicity: [1]

    
Attribute pitch
~~~~~~~~~~~~~~~
:Name: pitch
:Type: double
:Multiplicity: [1]

    
Attribute yaw
~~~~~~~~~~~~~
:Name: yaw
:Type: double
:Multiplicity: [1]


  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... x="double [1]" y="double [1]" z="double [1]" roll="double [1]" pitch="double [1]" yaw="double [1]"/> 


Complex Type: RosLaunch
^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: RosLaunch
:Abstract: no
:Documentation: 
  This type denotes a Ros Launchfile configuration.

Properties
""""""""""

    
    
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [1]
:Documentation: 
  The path to a ROSLaunch file

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [1]"/> 


Complex Type: SCXMLStateMachine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | StateMachine < SCXMLStateMachine (by extension)                                                           |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: SCXMLStateMachine
:Abstract: no
:Documentation: 
  This type denotes an SCXML state machine. SCXML is a W3C standard for state charts. However, state machines in this format are currently not run. State machines in SCXML are currently not interpreted.

Properties
""""""""""

    
    
      
        
          
Element 
~~~~~~~~
:Name: 
:Type: anyType
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no

        
        
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [0..1]

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [0..1]"> <!-- 'StateMachine' super type was not found in this schema. Some elements and attributes may be missing. -->
       <scxml> ... </scxml> [0..1]
    </...>


Complex Type: SMACHStateMachine
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | StateMachine < SMACHStateMachine (by extension)                                                           |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: SMACHStateMachine
:Abstract: no
:Documentation: 
  This type depicts a SMACH state machine. It is specified using a path to the source code of the state machine.

Properties
""""""""""

    
    
      
        
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [1]
:Documentation: 
  The path to an Python script that describes the state machine. This script has to have a variable with global scope that must have the name sm or stateMachine.

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [1]"> <!-- 'StateMachine' super type was not found in this schema. Some elements and attributes may be missing. -->
    </...>


Complex Type: StateMachine
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - SMACHStateMachine (by extension)                                                                        |
|             | - SCXMLStateMachine (by extension)                                                                        |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: StateMachine
:Abstract: yes
:Documentation: 
  This abstract type depicts a state machine. Currently, State Machines in SMACH or SCXML are supported, though state machines in SCXML are currently ignored.

Properties
""""""""""

    
    
Attribute id
~~~~~~~~~~~~
:Name: id
:Type: string
:Multiplicity: [1]
:Documentation: 
  Any state machine must have an identifier. This identifier is used to communicate with the state machine and therefore must be an identifier.

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... id="string [1]"/> 


Complex Type: VisualModel
^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: VisualModel
:Abstract: no
:Documentation: 
  This type defines a visual model (for example for the robot) as used in the frontend.

Properties
""""""""""

    
    
      
Element visualPose
~~~~~~~~~~~~~~~~~~
:Name: visualPose
:Type: RobotPose
:Multiplicity: [1]
:Nillable: no
:Abstract: no

    
    
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [1]

    
Attribute scale
~~~~~~~~~~~~~~~
:Name: scale
:Type: double
:Multiplicity: [0..1]

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [1]" scale="double [0..1]"> 
       <visualPose> RobotPose </visualPose> [1]
    </...>


Simple Type: ConfType
^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ConfType
:Content: Union of following types: ConfTypeEnumerationstring
:Documentation: 
  This type denotes a configuration type which can be a standard configuration type or a custom type. The latter is just any string.


Simple Type: ConfTypeEnumeration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < ConfTypeEnumeration (by restriction)                                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ConfTypeEnumeration
:Content: Base XSD Type: stringvalue comes from list: {'3d-settings'}
:Documentation: 
  This enumeration lists the standard configuration types used in the NRP.

Members
"""""""
* 3d-settings


Simple Type: MaturityType
^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < MaturityType (by restriction)                                                                    |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: MaturityType
:Content: Base XSD Type: stringvalue comes from list: {'development'|'production'}
:Documentation: 
  This type denotes a maturity of an experiment. It can either be development or production.

Members
"""""""
* development
* production


Simple Type: PhysicsEngine
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < PhysicsEngine (by restriction)                                                                   |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: PhysicsEngine
:Content: Base XSD Type: stringvalue comes from list: {'ode'|'opensim'}
:Documentation: 
  This enumeration contains the physics engines supported by the NRP. This includes the standard physics engine ODE and OpenSim.

Members
"""""""
* ode
* opensim


Simple Type: ThumbnailFile
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < ThumbnailFile (by restriction)                                                                   |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ThumbnailFile
:Content: Base XSD Type: stringpattern = [a-zA-Z0-9\._\-/]*\.(png|gif|jp[e]?g)
:Documentation: 
  This type denotes a path to an image file. The supported extensions are .png, .jpg, .jpeg and .gif. The file name must not contain whitespaces.

