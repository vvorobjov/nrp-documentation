.. index:: pair: page; Transceiver Function Schema
.. _doxid-transceiver_function_schema:

Transceiver Function Schema
===========================

The TransceiverFunctions are responsible for exchanging data between engines. They are simple python scripts that take datapacks as input (i.e. data, either as received from engines, or pre-processed by preprocessing functions), performs some computation on those datapacks, and output other datapacks to be consumed by a given engine. Additional information can be found :ref:`here <doxid-transceiver_function>`. In order to use them in an experiment they must be added in the simulation configuration file.

Below is a list of all the parameters needed to configure a :ref:`TransceiverFunction <doxid-class_transceiver_function>`.



.. _doxid-transceiver_function_schema_1transceiver_function_schema_parameters:

Parameters
~~~~~~~~~~

========  ==============================================================  =======  =======  ========  =====  
Name      Description                                                     Type     Default  Required  Array  
========  ==============================================================  =======  =======  ========  =====  
Name      Name of TF                                                      string            X                
FileName  Name of file containing the transceiver function Python script  string            X                
IsActive  Tells if this TF is active. Only active TFs will be executed    boolean  True                      
========  ==============================================================  =======  =======  ========  =====





.. _doxid-transceiver_function_schema_1transceiver_function_schema_example:

Example
~~~~~~~

.. ref-code-block:: cpp

	{
	    "Name": "tf_1",
	    "FileName": "tf_1.py"
	}





.. _doxid-transceiver_function_schema_1transceiver_function_schema_schema:

Schema
~~~~~~

.. ref-code-block:: cpp

	{
	  "$schema": "http://json-schema.org/draft-07/schema#",
	  "title": "Transceiver Function",
	  "description": "Transceiver Function configuration schema",
	  "$id": "#TransceiverFunction",
	  "type": "object",
	  "properties" : {
	    "Name" : {
	      "type" : "string",
	      "description": "Name of TF"
	    },
	    "FileName" : {
	      "type" : "string",
	      "description": "Name of file containing the transceiver function python script"
	    },
	    "IsActive" : {
	      "type" : "boolean",
	      "default": true,
	      "description": "Tells if this TF is active. Only active TFs will be executed"
	    }
	  },
	  "required" : ["Name", "FileName"]
	}

