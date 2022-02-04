.. index:: pair: struct; PythonConfigConst
.. _doxid-struct_python_config_const:

struct PythonConfigConst
========================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <python_config.h>
	
	struct PythonConfigConst {
		// fields
	
		static constexpr char :target:`EngineType<doxid-struct_python_config_const_1a48f0b700a0509b2c5f196e4e13acd263>`[] = "python_json";
		static constexpr char :target:`EngineSchema<doxid-struct_python_config_const_1ab743072261e4169588160c20a2e20513>`[] = "https://neurorobotics.net/engines/:ref:`engine_python.json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`#/python_json";
		static constexpr std::string_view :ref:`InitFileExecStatus<doxid-struct_python_config_const_1a0990040feaa24ba9469e8260ef39dea3>` = "InitExecStatus";
		static constexpr std::string_view :ref:`ResetExecStatus<doxid-struct_python_config_const_1a1bed9f27f7739406a2bc6a85d52d76de>` = "ResetExecStatus";
		static constexpr std::string_view :ref:`ErrorMsg<doxid-struct_python_config_const_1ad871f9f786d12c5d0b1e7c9311f42248>` = "Message";
	};
.. _details-struct_python_config_const:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. index:: pair: variable; InitFileExecStatus
.. _doxid-struct_python_config_const_1a0990040feaa24ba9469e8260ef39dea3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view InitFileExecStatus = "InitExecStatus"

After the server executes the init file, this status flag will either be 1 for success or 0 for fail. If the execution fails, a JSON message with more details will be passed as well (under ErrorMsg).

.. index:: pair: variable; ResetExecStatus
.. _doxid-struct_python_config_const_1a1bed9f27f7739406a2bc6a85d52d76de:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view ResetExecStatus = "ResetExecStatus"

After the server resets, this status flag will either be 1 for success or 0 for fail. If the execution fails, a JSON message with more details will be passed as well (under ErrorMsg).

.. index:: pair: variable; ErrorMsg
.. _doxid-struct_python_config_const_1ad871f9f786d12c5d0b1e7c9311f42248:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view ErrorMsg = "Message"

If the init file could not be parsed, the python error message will be stored under this JSON property name.

