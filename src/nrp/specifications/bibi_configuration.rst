.. _bibi-specification:

BIBI Model Specification
========================

.. todo:: Add author/responsible

Schema Document Properties
--------------------------
:Target Namespace: http://schemas.humanbrainproject.eu/SP10/2014/BIBI
:Element Qualifiers: By default, local element declarations belong to this schema's target namespace.
:Attribute Qualifiers: By default, local attribute declarations have no namespace.


Declared Namespaces
-------------------

+-------------------+---------------------------------------------------------------------------------------------------------+
| Prefix            | Namespace                                                                                               |
+===================+=========================================================================================================+
| (default)         | http://schemas.humanbrainproject.eu/SP10/2014/BIBI                                                      |
+-------------------+---------------------------------------------------------------------------------------------------------+
| xml               | http://www.w3.org/XML/1998/namespace                                                                    |
+-------------------+---------------------------------------------------------------------------------------------------------+
| xs                | http://www.w3.org/2001/XMLSchema                                                                        |
+-------------------+---------------------------------------------------------------------------------------------------------+
| xsd               | http://www.w3.org/2001/XMLSchema                                                                        |
+-------------------+---------------------------------------------------------------------------------------------------------+



Global Declarations
-------------------
   
Element: bibi
^^^^^^^^^^^^^

:Name: bibi
:Type: BIBIConfiguration
:Multiplicity: [1]
:Nillable: no
:Abstract: no

XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <bibi> 
       <timestep> TimeStep </timestep> [0..1]
       <brainModel> BrainModel </brainModel> [1]
       <bodyModel> SDFFilename </bodyModel> [1]
       <extRobotController> ScriptFilename </extRobotController> [0..1]
       <configuration> ConfFile </configuration> [0..*]
       <connectors> NeuronConnector </connectors> [0..*]
       <synapseDynamics> SynapseDynamics </synapseDynamics> [0..*]
       <transferFunction> TransferFunction </transferFunction> [0..*]
    </bibi>



Global Definitions
------------------
   
Complex Type: Add
^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Operator (by extension) < Add (by extension)                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Add
:Abstract: no
:Documentation: 
  The sum of all operands

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <operand> FlowExpression </operand> [2..*]
    </...>


Complex Type: AllToAllConnector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronConnector < AllToAllConnector (by extension)                                                        |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: AllToAllConnector
:Abstract: no
:Documentation: 
  This connector type is obsolete.

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... weights="double [0..1]" delays="double [0..1]" name="string [0..1]"/> 


Complex Type: Argument
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Argument
:Abstract: no
:Documentation: 
  A named argument

Properties
""""""""""

    
    
      
Element value
~~~~~~~~~~~~~
:Name: value
:Type: FlowExpression
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The value passed for this argument

    
    
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of this argument

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]"> 
       <value> FlowExpression </value> [1]
    </...>


Complex Type: ArgumentReference
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < ArgumentReference (by extension)                                                         |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ArgumentReference
:Abstract: no
:Documentation: 
  A reference to an argument, either a device or a local variable

Properties
""""""""""

    
    
      
        
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of the device or local variable

        
Attribute property
~~~~~~~~~~~~~~~~~~
:Name: property
:Type: string
:Multiplicity: [0..1]
:Documentation: 
  If specified, only a property of the local variable is referenced. Otherwise, the value itself (or the default property of a device) is selected.

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]" property="string [0..1]"/> 


Complex Type: BIBIConfiguration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: BIBIConfiguration
:Abstract: no
:Documentation: 
  This class represents the root of the BIBI configuration.

Properties
""""""""""

    
    
      
Element timestep
~~~~~~~~~~~~~~~~
:Name: timestep
:Type: TimeStep
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  If specified, the CLE uses a different timestep than the default timestep of 20ms. The timestep is specified in milliseconds and depicts the time between two successive loops of the CLE in simulation time.

      
Element brainModel
~~~~~~~~~~~~~~~~~~
:Name: brainModel
:Type: BrainModel
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The brain model depicts a path to the neural network model.

      
Element bodyModel
~~~~~~~~~~~~~~~~~
:Name: bodyModel
:Type: SDFFilename
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The path to the robot model that should be used. This can either be a path to an SDF model or a path to a zip file containing all required assets for a robot. This zip file must have a file model.sdf at the root of the archive.

      
Element extRobotController
~~~~~~~~~~~~~~~~~~~~~~~~~~
:Name: extRobotController
:Type: ScriptFilename
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  A path to an external robot controller. If specified, the robot controller is started when the simulation begins and stopped when the simulation is over. Therefore, the path must be a path to a shell script that offers a function start and a function stop.

      
Element configuration
~~~~~~~~~~~~~~~~~~~~~
:Name: configuration
:Type: ConfFile
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  The configuration entries of an experiment depict additional files required for the simulation of experiments using this BIBI configuration.

      
Element connectors
~~~~~~~~~~~~~~~~~~
:Name: connectors
:Type: NeuronConnector
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  A list of connectors. This can be useful when specifying transfer functions 

      
Element synapseDynamics
~~~~~~~~~~~~~~~~~~~~~~~
:Name: synapseDynamics
:Type: SynapseDynamics
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  A list of synapse dynamics. Such a synapse dynamic can be referenced later on in neural network devices.

      
Element transferFunction
~~~~~~~~~~~~~~~~~~~~~~~~
:Name: transferFunction
:Type: TransferFunction
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  The transfer functions that are used to couple a neural network to robot

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <timestep> TimeStep </timestep> [0..1]
       <brainModel> BrainModel </brainModel> [1]
       <bodyModel> SDFFilename </bodyModel> [1]
       <extRobotController> ScriptFilename </extRobotController> [0..1]
       <configuration> ConfFile </configuration> [0..*]
       <connectors> NeuronConnector </connectors> [0..*]
       <synapseDynamics> SynapseDynamics </synapseDynamics> [0..*]
       <transferFunction> TransferFunction </transferFunction> [0..*]
    </...>


Complex Type: BIBITransferFunction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | TransferFunction < BIBITransferFunction (by extension)                                                    |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - Robot2Neuron (by extension)                                                                             |
|             | - Neuron2Monitor (by restriction)                                                                         |
|             | - Neuron2Robot (by extension)                                                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: BIBITransferFunction
:Abstract: yes
:Documentation: 
  This type denotes the abstract base type of Transfer Functions specified entirely in the BIBI model, in XML

Properties
""""""""""

    
    
      
        
          
Element local
~~~~~~~~~~~~~
:Name: local
:Type: Local
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  This denotes the local variables of this transfer function.

          
            
Element device
~~~~~~~~~~~~~~
:Name: device
:Type: DeviceChannel
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  This denotes device channels, connections of the transfer function to the neural network using exactly one device.

            
Element deviceGroup
~~~~~~~~~~~~~~~~~~~
:Name: deviceGroup
:Type: DeviceGroupChannel
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  This denotes the device group channels, connections of transfer functions to the neural network using a one-dimensional array of devices.

          
          
Element topic
~~~~~~~~~~~~~
:Name: topic
:Type: TopicChannel
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  This denotes the connections of the transfer function to robot control channels.

        
        
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of the transfer function. This is used to identify the transfer function in order to update or delete it in a running simulation.

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]"> 
       <local> Local </local> [0..*]
       <device> DeviceChannel </device> [1]
       <deviceGroup> DeviceGroupChannel </deviceGroup> [1]
       <topic> TopicChannel </topic> [0..*]
    </...>


Complex Type: BrainModel
^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: BrainModel
:Abstract: no
:Documentation: 
  A neural network description as used in the CLE

Properties
""""""""""

    
    
      
Element file
~~~~~~~~~~~~
:Name: file
:Type: BrainFilename
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  A path to the neural network file.

      
Element populations
~~~~~~~~~~~~~~~~~~~
:Name: populations
:Type: MultiNeuronSelector
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  The populations in this field are the explicitly defined populations. Each of this population is defined as a view of an assumed 'circuit' population.

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <file> BrainFilename </file> [1]
       <populations> MultiNeuronSelector </populations> [0..*]
    </...>


Complex Type: Call
^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Call (by extension)                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Call
:Abstract: no
:Documentation: 
  A call to a static method

Properties
""""""""""

    
    
      
        
          
Element argument
~~~~~~~~~~~~~~~~
:Name: argument
:Type: Argument
:Multiplicity: [1..*]
:Nillable: no
:Abstract: no
:Documentation: 
  Named arguments that are passed to the selected method

        
        
Attribute type
~~~~~~~~~~~~~~
:Name: type
:Type: string
:Multiplicity: [1]
:Documentation: 
  A reference to the static method. This is specified as a full path of a Python function, including both the path of the module and the name of the function. For this to work, the function must be static, i.e. a global function on that module or a static class function.

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... type="string [1]"> 
       <argument> Argument </argument> [1..*]
    </...>


Complex Type: ChainSelector
^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronGroupSelector < ChainSelector (by extension)                                                        |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ChainSelector
:Abstract: no
:Documentation: 
  A chain of neurons or neuron groups

Properties
""""""""""

    
    
      
        
          
Element neurons
~~~~~~~~~~~~~~~
:Name: neurons
:Type: NeuronSelector
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  Single neuron connections such as single neurons

          
Element connectors
~~~~~~~~~~~~~~~~~~
:Name: connectors
:Type: NeuronGroupSelector
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no
:Documentation: 
  Existing groups of neurons

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <neurons> NeuronSelector </neurons> [0..*]
       <connectors> NeuronGroupSelector </connectors> [0..*]
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
  This type denotes an additional configuration entry that consists of a file and a purpose.

Properties
""""""""""

    
    
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: string
:Multiplicity: [1]
:Documentation: 
  The source of a configuration entry is a path to a file that contains the necessary information. The path is relative to the BIBI model.

    
Attribute type
~~~~~~~~~~~~~~
:Name: type
:Type: ConfType
:Multiplicity: [1]
:Documentation: 
  The type of a configuration entry denotes the purpose how this entry is used. This is used to decouple the purpose of a configuration entry from the file name.

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="string [1]" type="ConfType [1]"/> 


Complex Type: Constant
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Constant (by extension)                                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Constant
:Abstract: no
:Documentation: 
  A constant as a flow element

Properties
""""""""""

    
    
      
        
Attribute value
~~~~~~~~~~~~~~~
:Name: value
:Type: double
:Multiplicity: [1]
:Documentation: 
  The value for this constant

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... value="double [1]"/> 


Complex Type: ConstantString
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < ConstantString (by extension)                                                            |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ConstantString
:Abstract: no
:Documentation: 
  A constant string

Properties
""""""""""

    
    
      
        
Attribute value
~~~~~~~~~~~~~~~
:Name: value
:Type: string
:Multiplicity: [1]
:Documentation: 
  The value of this string constant

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... value="string [1]"/> 


Complex Type: DeviceChannel
^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: DeviceChannel
:Abstract: no
:Documentation: 
  This type denotes a connection of a transfer function to a neural network

Properties
""""""""""

    
    
      
Element neurons
~~~~~~~~~~~~~~~
:Name: neurons
:Type: NeuronSelector
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  This specifies the neurons that should be connected to this neural connector device

      
        
Element connector
~~~~~~~~~~~~~~~~~
:Name: connector
:Type: NeuronConnector
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  Additional information on the connection to the neurons

        
Element connectorRef
~~~~~~~~~~~~~~~~~~~~
:Name: connectorRef
:Type: NeuronConnectorRef
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  A reference to a reusable connector

      
      
        
Element synapseDynamics
~~~~~~~~~~~~~~~~~~~~~~~
:Name: synapseDynamics
:Type: SynapseDynamics
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  Additional information on the dynamics of the connection of this device to the neural network

        
Element synapseDynamicsRef
~~~~~~~~~~~~~~~~~~~~~~~~~~
:Name: synapseDynamicsRef
:Type: SynapseDynamicsRef
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  A reference to a reusable synapse dynamics

      
      
Element target
~~~~~~~~~~~~~~
:Name: target
:Type: NeuronTarget
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The target of this connection. This configuration is useful in particular for spike source devices such as Poisson generators. By default, these devices are excitatory but they can be configured to inhibit connected neurons.

      
Element body
~~~~~~~~~~~~
:Name: body
:Type: FlowExpression
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  This element is only meaningful for spike sources. It depicts the value to which the device should be configured.

    
    
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of this device channel

    
Attribute type
~~~~~~~~~~~~~~
:Name: type
:Type: DeviceType
:Multiplicity: [1]
:Documentation: 
  The type of the neural network connection specified with this device channel

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]" type="DeviceType [1]"> 
       <neurons> NeuronSelector </neurons> [1]
       <connector> NeuronConnector </connector> [1]
       <connectorRef> NeuronConnectorRef </connectorRef> [1]
       <synapseDynamics> SynapseDynamics </synapseDynamics> [1]
       <synapseDynamicsRef> SynapseDynamicsRef </synapseDynamicsRef> [1]
       <target> NeuronTarget </target> [0..1]
       <body> FlowExpression </body> [0..1]
    </...>


Complex Type: DeviceGroupChannel
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: DeviceGroupChannel
:Abstract: no
:Documentation: 
  This type denotes a connection of a transfer function to a neural network using an array of devices

Properties
""""""""""

    
    
      
Element neurons
~~~~~~~~~~~~~~~
:Name: neurons
:Type: NeuronGroupSelector
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  This specifies the neurons that should be connected to this neural connector device

      
        
Element connector
~~~~~~~~~~~~~~~~~
:Name: connector
:Type: NeuronConnector
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  Additional information on the connection to the neurons

        
Element connectorRef
~~~~~~~~~~~~~~~~~~~~
:Name: connectorRef
:Type: NeuronConnectorRef
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  A reference to a reusable connector

      
      
        
Element synapseDynamics
~~~~~~~~~~~~~~~~~~~~~~~
:Name: synapseDynamics
:Type: SynapseDynamics
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  Additional information on the dynamics of the connection of this device to the neural network

        
Element synapseDynamicsRef
~~~~~~~~~~~~~~~~~~~~~~~~~~
:Name: synapseDynamicsRef
:Type: SynapseDynamicsRef
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  A reference to a reusable synapse dynamics

      
      
Element target
~~~~~~~~~~~~~~
:Name: target
:Type: NeuronTarget
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The target of this connection. This configuration is useful in particular for spike source devices such as Poisson generators. By default, these devices are excitatory but they can be configured to inhibit connected neurons.

      
Element body
~~~~~~~~~~~~
:Name: body
:Type: FlowExpression
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  This element is only meaningful for spike sources. It depicts the value to which the device should be configured.

    
    
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of this device group channel

    
Attribute type
~~~~~~~~~~~~~~
:Name: type
:Type: DeviceType
:Multiplicity: [1]
:Documentation: 
  The type of the neural network connection specified with this device group channel

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]" type="DeviceType [1]"> 
       <neurons> NeuronGroupSelector </neurons> [1]
       <connector> NeuronConnector </connector> [1]
       <connectorRef> NeuronConnectorRef </connectorRef> [1]
       <synapseDynamics> SynapseDynamics </synapseDynamics> [1]
       <synapseDynamicsRef> SynapseDynamicsRef </synapseDynamicsRef> [1]
       <target> NeuronTarget </target> [0..1]
       <body> FlowExpression </body> [0..1]
    </...>


Complex Type: Divide
^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Operator (by extension) < Divide (by restriction)                                        |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Divide
:Abstract: no
:Documentation: 
  The quotient of two operands

Properties
""""""""""

    
    
      
        
          
Element operand
~~~~~~~~~~~~~~~
:Name: operand
:Type: FlowExpression
:Multiplicity: [2..2]
:Nillable: no
:Abstract: no
:Documentation: 
  The arguments of the operator expression

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <operand> FlowExpression </operand> [2..2]
    </...>


Complex Type: FixedNumberPreConnector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronConnector < FixedNumberPreConnector (by extension)                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: FixedNumberPreConnector
:Abstract: no
:Documentation: 
  This connector type is obsolete.

Properties
""""""""""

    
    
      
        
Attribute count
~~~~~~~~~~~~~~~
:Name: count
:Type: positiveInteger
:Multiplicity: [1]

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... weights="double [0..1]" delays="double [0..1]" name="string [0..1]" count="positiveInteger [1]"/> 


Complex Type: FlowExpression
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+--------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                   |
+-------------+--------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                        |
|             | - Scale (by extension)                                                                                 |
|             | - Call (by extension)                                                                                  |
|             | - Operator (by extension)                                                                              |
|             | - Add (by extension)                                                                                   |
|             | - Subtract (by restriction)                                                                            |
|             | - Multiply (by extension)                                                                              |
|             | - Divide (by restriction)                                                                              |
|             | - Min (by extension)                                                                                   |
|             | - Max (by extension)                                                                                   |
|             | - SimulationStep (by extension)                                                                        |
|             | - ArgumentReference (by extension)                                                                     |
|             | - Constant (by extension)                                                                              |
|             | - ConstantString (by extension)                                                                        |
+-------------+--------------------------------------------------------------------------------------------------------+

:Name: FlowExpression
:Abstract: yes
:Documentation: 
  The abstract base class for an information flow expression. In the scope of the Transfer functions, an information flow is an expression without any control flow.

Properties
""""""""""

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <.../> 


Complex Type: Index
^^^^^^^^^^^^^^^^^^^

+-------------+--------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelector < Index (by extension)                                                                  |
+-------------+--------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                   |
+-------------+--------------------------------------------------------------------------------------------------------+

:Name: Index
:Abstract: no
:Documentation: 
  Selection of exactly one neuron using an index of a base population

Properties
""""""""""

    
    
      
        
Attribute index
~~~~~~~~~~~~~~~
:Name: index
:Type: nonNegativeInteger
:Multiplicity: [1]
:Documentation: 
  The index of the selected neuron within its population

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... population="string [1]" index="nonNegativeInteger [1]"/> 


Complex Type: IndexTemplate
^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+--------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelectorTemplate < IndexTemplate (by extension)                                                  |
+-------------+--------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                   |
+-------------+--------------------------------------------------------------------------------------------------------+

:Name: IndexTemplate
:Abstract: no
:Documentation: 
  A template for an index-based neuron selection

Properties
""""""""""

    
    
      
        
Attribute index
~~~~~~~~~~~~~~~
:Name: index
:Type: TemplatePattern
:Multiplicity: [1]
:Documentation: 
  The template for the index to access the neurons

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... index="TemplatePattern [1]"/> 


Complex Type: List
^^^^^^^^^^^^^^^^^^

+-------------+--------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelector < MultiNeuronSelector (by extension) < List (by extension)                              |
+-------------+--------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                   |
+-------------+--------------------------------------------------------------------------------------------------------+

:Name: List
:Abstract: no
:Documentation: 
  Selection of a list of neurons using their indices

Properties
""""""""""

    
    
      
        
          
Element element
~~~~~~~~~~~~~~~
:Name: element
:Type: nonNegativeInteger
:Multiplicity: [1..*]
:Nillable: no
:Abstract: no
:Documentation: 
  The indices of selected neurons

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... population="string [1]"> 
       <element> nonNegativeInteger </element> [1..*]
    </...>


Complex Type: ListTemplate
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+--------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelectorTemplate < ListTemplate (by extension)                                                   |
+-------------+--------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                   |
+-------------+--------------------------------------------------------------------------------------------------------+

:Name: ListTemplate
:Abstract: no
:Documentation: 
  A template for a list-based neuron selection

Properties
""""""""""

    
    
      
        
          
Element element
~~~~~~~~~~~~~~~
:Name: element
:Type: TemplatePattern
:Multiplicity: [1..*]
:Nillable: no
:Abstract: no
:Documentation: 
  Templates for the indices of selected neurons

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <element> TemplatePattern </element> [1..*]
    </...>


Complex Type: Local
^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Local
:Abstract: no
:Documentation: 
  A local variable

Properties
""""""""""

    
    
      
Element body
~~~~~~~~~~~~
:Name: body
:Type: FlowExpression
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The initial value for this local variable

    
    
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of the local variable

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]"> 
       <body> FlowExpression </body> [1]
    </...>


Complex Type: MapSelector
^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+--------------------------------------------------------------------------------------------------------+
| Super-types | NeuronGroupSelector < MapSelector (by extension)                                                       |
+-------------+--------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                   |
+-------------+--------------------------------------------------------------------------------------------------------+

:Name: MapSelector
:Abstract: no
:Documentation: 
  An indexed mapping of neurons to neuron groups. As index, either a number or a population may be used. In the latter case, the size of the given population is used as count.

Properties
""""""""""

    
    
      
        
          
            
Element count
~~~~~~~~~~~~~
:Name: count
:Type: positiveInteger
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The number of neural network connections contained in this indexed mapping

            
Element source
~~~~~~~~~~~~~~
:Name: source
:Type: MultiNeuronSelector
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The source population. If possibility is used, the indexed group consists of one neuron selection per neuron in the source group

          
          
Element pattern
~~~~~~~~~~~~~~~
:Name: pattern
:Type: NeuronSelectorTemplate
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The pattern that shall be used to select neurons

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <count> positiveInteger </count> [1]
       <source> MultiNeuronSelector </source> [1]
       <pattern> NeuronSelectorTemplate </pattern> [1]
    </...>


Complex Type: Max
^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Operator (by extension) < Max (by extension)                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Max
:Abstract: no
:Documentation: 
  The maximum of the provided values

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <operand> FlowExpression </operand> [2..*]
    </...>


Complex Type: Min
^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Operator (by extension) < Min (by extension)                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Min
:Abstract: no
:Documentation: 
  The minimum of the provided values

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <operand> FlowExpression </operand> [2..*]
    </...>


Complex Type: MultiNeuronSelector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelector < MultiNeuronSelector (by extension)                                                       |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - Range (by extension)                                                                                    |
|             | - List (by extension)                                                                                     |
|             | - Population (by extension)                                                                               |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: MultiNeuronSelector
:Abstract: yes
:Documentation: 
  
        The abstract base class of selections of multiple neurons
      

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... population="string [1]"/> 


Complex Type: Multiply
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Operator (by extension) < Multiply (by extension)                                        |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Multiply
:Abstract: no
:Documentation: 
  The product of all operands

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <operand> FlowExpression </operand> [2..*]
    </...>


Complex Type: Neuron2Monitor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | TransferFunction < BIBITransferFunction (by extension) < Neuron2Monitor (by restriction)                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Neuron2Monitor
:Abstract: no
:Documentation: 
  A NeuronMonitor is a special class of transfer functions that monitors neural network populations. Connections to robot control topics or device groups are not allowed.

Properties
""""""""""

    
    
      
        
          
Element local
~~~~~~~~~~~~~
:Name: local
:Type: Local
:Multiplicity: [0..*]
:Nillable: no
:Abstract: no

          
            
Element device
~~~~~~~~~~~~~~
:Name: device
:Type: DeviceChannel
:Multiplicity: [1]
:Nillable: no
:Abstract: no

          
        
        
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]"> 
       <local> Local </local> [0..*]
       <device> DeviceChannel </device> [1]
    </...>


Complex Type: Neuron2Robot
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | TransferFunction < BIBITransferFunction (by extension) < Neuron2Robot (by extension)                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Neuron2Robot
:Abstract: no
:Documentation: 
  A Neuron2Robot transfer function is a transfer function whose primary purpose is to extract information from the neural network and use this information to control the robot using robot control messages

Properties
""""""""""

    
    
      
        
          
Element returnValue
~~~~~~~~~~~~~~~~~~~
:Name: returnValue
:Type: TopicChannel
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The return value topic channel of a Neuron2Robot transfer function is the channel to which control messages the return value of the Python function are sent

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]"> 
       <local> Local </local> [0..*]
       <device> DeviceChannel </device> [1]
       <deviceGroup> DeviceGroupChannel </deviceGroup> [1]
       <topic> TopicChannel </topic> [0..*]
       <returnValue> TopicChannel </returnValue> [0..1]
    </...>


Complex Type: NeuronConnector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - OneToOneConnector (by extension)                                                                        |
|             | - AllToAllConnector (by extension)                                                                        |
|             | - FixedNumberPreConnector (by extension)                                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: NeuronConnector
:Abstract: yes
:Documentation: 
  This type denotes a connector to other populations

Properties
""""""""""

    
    
Attribute weights
~~~~~~~~~~~~~~~~~
:Name: weights
:Type: double
:Multiplicity: [0..1]
:Documentation: 
  The weights of the connector denote the connections between the source neuron and the target neurons. If no weight is specified, the default weight of the neuron connection device is used.

    
Attribute delays
~~~~~~~~~~~~~~~~
:Name: delays
:Type: double
:Multiplicity: [0..1]
:Documentation: 
  The delays of the connector denote the delays of spike deliveries. If no delays are specified, the default delays of the neuron connection device is used.

    
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [0..1]
:Default Value: default
:Documentation: 
  The name of the connector for later reference.

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... weights="double [0..1]" delays="double [0..1]" name="string [0..1]"/> 


Complex Type: NeuronConnectorRef
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | anyType < NeuronConnectorRef (by restriction)                                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: NeuronConnectorRef
:Abstract: no
:Documentation: 
  A reference to an elsewhere defined neural connector

Properties
""""""""""

    
    
      
        
Attribute ref
~~~~~~~~~~~~~
:Name: ref
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of the referenced connector

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... ref="string [1]"> <!-- 'anyType' super type was not found in this schema. Some elements and attributes may be missing. -->
    </...>


Complex Type: NeuronGroupSelector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - ChainSelector (by extension)                                                                            |
|             | - MapSelector (by extension)                                                                              |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: NeuronGroupSelector
:Abstract: yes
:Documentation: 
  This type denotes an abstract group of neurons

Properties
""""""""""

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <.../> 


Complex Type: NeuronSelector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - Index (by extension)                                                                                    |
|             | - MultiNeuronSelector (by extension)                                                                      |
|             | - Range (by extension)                                                                                    |
|             | - List (by extension)                                                                                     |
|             | - Population (by extension)                                                                               |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: NeuronSelector
:Abstract: yes
:Documentation: 
  The abstract base class of neuron selectors

Properties
""""""""""

    
    
Attribute population
~~~~~~~~~~~~~~~~~~~~
:Name: population
:Type: string
:Multiplicity: [1]
:Documentation: 
  The population this neuron selector refers to

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... population="string [1]"/> 


Complex Type: NeuronSelectorTemplate
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - IndexTemplate (by extension)                                                                            |
|             | - RangeTemplate (by extension)                                                                            |
|             | - ListTemplate (by extension)                                                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: NeuronSelectorTemplate
:Abstract: yes
:Documentation: 
  A template for neuron selectors

Properties
""""""""""

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <.../> 


Complex Type: OneToOneConnector
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronConnector < OneToOneConnector (by extension)                                                        |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: OneToOneConnector
:Abstract: no
:Documentation: 
  
        This connector type is obsolete.
      

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... weights="double [0..1]" delays="double [0..1]" name="string [0..1]"/> 


Complex Type: Operator
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Operator (by extension)                                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - Add (by extension)                                                                                      |
|             | - Subtract (by restriction)                                                                               |
|             | - Multiply (by extension)                                                                                 |
|             | - Divide (by restriction)                                                                                 |
|             | - Min (by extension)                                                                                      |
|             | - Max (by extension)                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Operator
:Abstract: yes
:Documentation: 
  The abstract base class for an operator call based on a flow expression

Properties
""""""""""

    
    
      
        
          
Element operand
~~~~~~~~~~~~~~~
:Name: operand
:Type: FlowExpression
:Multiplicity: [2..*]
:Nillable: no
:Abstract: no
:Documentation: 
  The arguments of the operator expression

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <operand> FlowExpression </operand> [2..*]
    </...>


Complex Type: Population
^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelector < MultiNeuronSelector (by extension) < Population (by extension)                           |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Population
:Abstract: no
:Documentation: 
  Selection of an entire population of neurons

Properties
""""""""""

    
    
      
        
Attribute count
~~~~~~~~~~~~~~~
:Name: count
:Type: positiveInteger
:Multiplicity: [1]
:Documentation: 
  The size of the selected population. This is necessary for validation purposes where the neural network is not available.

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... population="string [1]" count="positiveInteger [1]"/> 


Complex Type: PythonTransferFunction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | TransferFunction < PythonTransferFunction (by extension)                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: PythonTransferFunction
:Abstract: no
:Documentation: 
  This type denotes a transfer function entirely specified in the Python DSL PyTF.

Properties
""""""""""

    
    
      
        
          
        
        
Attribute src
~~~~~~~~~~~~~
:Name: src
:Type: PythonFilename
:Multiplicity: [0..1]
:Documentation: 
  The 'src' attribute denotes the path of a python file that contains the entire transfer function. If this attribute is present, the actual contents of the transfer function element is ignored and only the contents of the specified Python file are taken into account.

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... src="PythonFilename [0..1]"> <!-- Mixed content -->Allow any elements from a namespace other than this schema's namespace (skip validation). [0..*]
    </...>


Complex Type: Range
^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelector < MultiNeuronSelector (by extension) < Range (by extension)                                |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Range
:Abstract: no
:Documentation: 
  Selection of a range of neurons from an existing population

Properties
""""""""""

    
    
      
        
Attribute from
~~~~~~~~~~~~~~
:Name: from
:Type: nonNegativeInteger
:Multiplicity: [1]
:Documentation: 
  The starting index from which neurons are selected

        
Attribute to
~~~~~~~~~~~~
:Name: to
:Type: nonNegativeInteger
:Multiplicity: [1]
:Documentation: 
  The stop index to which neurons are selected

        
Attribute step
~~~~~~~~~~~~~~
:Name: step
:Type: positiveInteger
:Multiplicity: [0..1]
:Documentation: 
  The step of the selection

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... population="string [1]" from="nonNegativeInteger [1]" to="nonNegativeInteger [1]" step="positiveInteger [0..1]"/> 


Complex Type: RangeTemplate
^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | NeuronSelectorTemplate < RangeTemplate (by extension)                                                     |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: RangeTemplate
:Abstract: no
:Documentation: 
  A template for the range-based neuron selection

Properties
""""""""""

    
    
      
        
Attribute from
~~~~~~~~~~~~~~
:Name: from
:Type: TemplatePattern
:Multiplicity: [1]
:Documentation: 
  A template for the start index of the selected range

        
Attribute to
~~~~~~~~~~~~
:Name: to
:Type: TemplatePattern
:Multiplicity: [1]
:Documentation: 
  A template for the end index of the selected range

        
Attribute step
~~~~~~~~~~~~~~
:Name: step
:Type: TemplatePattern
:Multiplicity: [0..1]
:Documentation: 
  A template for the step of the selected range

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... from="TemplatePattern [1]" to="TemplatePattern [1]" step="TemplatePattern [0..1]"/> 


Complex Type: Robot2Neuron
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | TransferFunction < BIBITransferFunction (by extension) < Robot2Neuron (by extension)                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Robot2Neuron
:Abstract: no
:Documentation: 
  A Robot2Neuron transfer function is a transfer function whose primary purpose is to translate information coming from robot sensors, transform it and push them into neural networks. 

Properties
""""""""""

    
    
      
        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]"> 
       <local> Local </local> [0..*]
       <device> DeviceChannel </device> [1]
       <deviceGroup> DeviceGroupChannel </deviceGroup> [1]
       <topic> TopicChannel </topic> [0..*]
    </...>


Complex Type: Scale
^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Scale (by extension)                                                                     |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Scale
:Abstract: no
:Documentation: 
  The scaling of an element by a constant factor

Properties
""""""""""

    
    
      
        
          
Element inner
~~~~~~~~~~~~~
:Name: inner
:Type: FlowExpression
:Multiplicity: [1]
:Nillable: no
:Abstract: no
:Documentation: 
  The inner flow expression

        
        
Attribute factor
~~~~~~~~~~~~~~~~
:Name: factor
:Type: double
:Multiplicity: [1]
:Documentation: 
  The factor by which the inner expression should be scaled

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... factor="double [1]"> 
       <inner> FlowExpression </inner> [1]
    </...>


Complex Type: SimulationStep
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < SimulationStep (by extension)                                                            |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: SimulationStep
:Abstract: no
:Documentation: 
  A reference to the simulation step

Properties
""""""""""

    
    
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <.../> 


Complex Type: Subtract
^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | FlowExpression < Operator (by extension) < Subtract (by restriction)                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: Subtract
:Abstract: no
:Documentation: 
  The difference between two operands

Properties
""""""""""

    
    
      
        
          
Element operand
~~~~~~~~~~~~~~~
:Name: operand
:Type: FlowExpression
:Multiplicity: [2..2]
:Nillable: no
:Abstract: no
:Documentation: 
  The arguments of the operator expression

        
      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <...> 
       <operand> FlowExpression </operand> [2..2]
    </...>


Complex Type: SynapseDynamics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - TsodyksMarkramMechanism (by extension)                                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: SynapseDynamics
:Abstract: yes
:Documentation: 
  This type denotes a reusable synapse dynamics configuration

Properties
""""""""""

    
    
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [0..1]
:Default Value: default
:Documentation: 
  The name of the synapse dynamics configuration

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [0..1]"/> 


Complex Type: SynapseDynamicsRef
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | anyType < SynapseDynamicsRef (by restriction)                                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: SynapseDynamicsRef
:Abstract: no
:Documentation: 
  This type specifies a reference to a synapse dynamics configuration

Properties
""""""""""

    
    
      
        
Attribute ref
~~~~~~~~~~~~~
:Name: ref
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of the synapse dynamics configuration

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... ref="string [1]"> <!-- 'anyType' super type was not found in this schema. Some elements and attributes may be missing. -->
    </...>


Complex Type: TopicChannel
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: TopicChannel
:Abstract: no
:Documentation: 
  A connection of a transfer function to a robot control message topic

Properties
""""""""""

    
    
      
Element body
~~~~~~~~~~~~
:Name: body
:Type: FlowExpression
:Multiplicity: [0..1]
:Nillable: no
:Abstract: no
:Documentation: 
  The value that should be sent to the robot control topic. If this element is present, then the channel is published to. Otherwise, the channel subscribes to the selected topic.

    
    
Attribute name
~~~~~~~~~~~~~~
:Name: name
:Type: string
:Multiplicity: [1]
:Documentation: 
  The name of the robot topic channel

    
Attribute topic
~~~~~~~~~~~~~~~
:Name: topic
:Type: RobotTopicAddress
:Multiplicity: [1]
:Documentation: 
  The actual topic address, for example '/husky/cmd_vel'

    
Attribute type
~~~~~~~~~~~~~~
:Name: type
:Type: string
:Multiplicity: [1]
:Documentation: 
  The type of the topic

  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [1]" topic="RobotTopicAddress [1]" type="string [1]"> 
       <body> FlowExpression </body> [0..1]
    </...>


Complex Type: TransferFunction
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   |                                                                                                           |
|             | - PythonTransferFunction (by extension)                                                                   |
|             | - BIBITransferFunction (by extension)                                                                     |
|             | - Robot2Neuron (by extension)                                                                             |
|             | - Neuron2Monitor (by restriction)                                                                         |
|             | - Neuron2Robot (by extension)                                                                             |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: TransferFunction
:Abstract: yes
:Documentation: 
  This is the abstract type for a transfer function specification. A transfer function may be specified either in XML or in Python. These specification options are reflected in subclasses of the abstract transfer function type.

Properties
""""""""""

    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <.../> 


Complex Type: TsodyksMarkramMechanism
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | SynapseDynamics < TsodyksMarkramMechanism (by extension)                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: TsodyksMarkramMechanism
:Abstract: no
:Documentation: 
  A synapse dynamics implementation based on the Tsodyks-Markram mechanism

Properties
""""""""""

    
    
      
        
Attribute u
~~~~~~~~~~~
:Name: u
:Type: double
:Multiplicity: [1]

        
Attribute tau_rec
~~~~~~~~~~~~~~~~~
:Name: tau_rec
:Type: double
:Multiplicity: [1]

        
Attribute tau_facil
~~~~~~~~~~~~~~~~~~~
:Name: tau_facil
:Type: double
:Multiplicity: [1]

      
    
  
XML Instance Representation
"""""""""""""""""""""""""""

.. code:: xml

    <... name="string [0..1]" u="double [1]" tau_rec="double [1]" tau_facil="double [1]"/> 


Simple Type: BrainFilename
^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: BrainFilename
:Content: Union of following types: H5FilenamePythonFilename
:Documentation: 
  This denotes the supported file types for neural network models. The current version only supports Python or H5 files for neural networks.


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
:Content: Base XSD Type: stringvalue comes from list: {'retina'|'brainvisualizer'}
:Documentation: 
  This enumeration lists the standard configuration types used in the NRP.

Members
"""""""
* retina
* brainvisualizer


Simple Type: DeviceType
^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < DeviceType (by restriction)                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: DeviceType
:Content: Base XSD Type: stringvalue comes from list: {'ACSource'|'DCSource'|'FixedFrequency'|'LeakyIntegratorAlpha'|'LeakyIntegratorExp'|'NCSource'|'Poisson'|'SpikeRecorder'|'PopulationRate'}
:Documentation: 
  The device types supported by the CLE

Members
"""""""
* ACSource: The current generators for direct current, alternating current or noisy current do not generate spikes but inject currents of the specified type into all of the connected neurons. These devices receive the amplitude of the generated current as inputs. The ACSource injects alternating currents. 
* DCSource: The DCSource type is similar to the ACSource but injects directed currents.
* FixedFrequency: A fixed frequency generator deterministically generates spikes at a given frequency. Here, the frequency is set as a parameter and can be adjusted to sensory inputs. Unlike the other spike generators, this device type is not directly implemented in neuronal simulators but can be implemented by connecting a current generator with an integrate-and-fire neuron. 
* LeakyIntegratorAlpha: The concept of leaky integrators is to simply integrate spikes coming from a neuron under observation and add a leak term to it. The rationale behind this is that in spiking neuronal networks, the membrane potential is highly fragile. Shortly after a spike has been issued, the membrane potential is reset and therefore, it has a high importance whether any measurement is taken before or after a neuron spikes. Therefore, we augment the neuronal network with an additional leaky integrate-and-fire neuron with an infinite threshold potential (so that it never spikes) and measure the membrane potential of this neuron. The result is much less fragile and therefore appropriate to be used for robot control signals. This version of leaky integrators has an alpha-shaped post-synaptic current.
* LeakyIntegratorExp: This device type is similar to LeakyIntegratorAlpha but has an exponentially shaped post-synaptic current.
* NCSource: The noisy current generator is rather a tool to test whether the neuronal network currently simulated is robust with regard to noise rather than being a good choice to encode sensory inputs.
* Poisson: A Poisson generator issues spikes according to a Poisson distribution. Here, the inverse of the lambda parameter can be set in accordance to sensory inputs. This inverse reflects the rate in which spikes are generated by this device. 
* SpikeRecorder: The simplest thing a spike sink can do is to simply record all spikes issued to a neuron under observation. However, this has two major drawbacks. At first, the communication overhead is increased since all spikes are transmitted between the neuronal simulation and the transfer function but more importantly the transfer function has to make sense of this series of spikes. This allows great flexibility as this approach is extensible, but it is not suited for general use.
* PopulationRate: Another very common pattern is to simply take the average incoming spike rate of a neuron or a range of neurons. This is again relatively stable and can be used for translation into robot control signals.


Simple Type: H5Filename
^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < H5Filename (by restriction)                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: H5Filename
:Content: Base XSD Type: stringpattern = [a-zA-Z0-9\._/]*\.h5
:Documentation: 
  This type denotes a path to an H5 file.


Simple Type: NeuronTarget
^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < NeuronTarget (by restriction)                                                                    |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: NeuronTarget
:Content: Base XSD Type: stringvalue comes from list: {'Inhibitory'|'Excitatory'}
:Documentation: 
  The target of a neural connection

Members
"""""""
* Inhibitory: Inhibitory means that the artificial synapse inhibits the target neuron, i.e. lowers its membrane potential.
* Excitatory: Excitatory means that the artificial synapse excites the target neuron, i.e. the membrane potential raises


Simple Type: PythonFilename
^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < PythonFilename (by restriction)                                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: PythonFilename
:Content: Base XSD Type: stringpattern = [a-zA-Z0-9\._/]*\.py
:Documentation: 
  This type denotes a path to a Python file.


Simple Type: RobotTopicAddress
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < RobotTopicAddress (by restriction)                                                               |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: RobotTopicAddress
:Content: Base XSD Type: stringpattern = (/[a-zA-Z0-9\_-]+)+(/)?
:Documentation: 
  This type denotes a valid address of a robot control topic


Simple Type: ScriptFilename
^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < ScriptFilename (by restriction)                                                                  |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: ScriptFilename
:Content: Base XSD Type: stringpattern = [a-zA-Z0-9\._/]*\.sh
:Documentation: 
  This type denotes a path to a script file.


Simple Type: SDFFilename
^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < SDFFilename (by restriction)                                                                     |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: SDFFilename
:Content: Base XSD Type: stringpattern = [a-zA-Z0-9\._/]*\.(sdf|zip)
:Documentation: 
  This type denotes a path to an SDF (or Zip) file


Simple Type: TemplatePattern
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | string < TemplatePattern (by restriction)                                                                 |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: TemplatePattern
:Content: Base XSD Type: stringpattern = (\(\s*)*(i|\d+)(\s*(\+|\*)\s*(\(\s*)*(i|\d+)\s*|\))*
:Documentation: 
  A regular expression denoting simple arithmetic index computations based on an index called i


Simple Type: TimeStep
^^^^^^^^^^^^^^^^^^^^^

+-------------+-----------------------------------------------------------------------------------------------------------+
| Super-types | positiveInteger < TimeStep (by restriction)                                                               |
+-------------+-----------------------------------------------------------------------------------------------------------+
| Sub-types   | None                                                                                                      |
+-------------+-----------------------------------------------------------------------------------------------------------+

:Name: TimeStep
:Content: Base XSD Type: positiveIntegervalue <= 1000
:Documentation: 
  The timestep type of the CLE. This is a positive number in milliseconds. The maximum allowed value is 1000.

