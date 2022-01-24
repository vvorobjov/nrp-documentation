.. index:: pair: struct; NestConfigConst
.. _doxid-struct_nest_config_const:

struct NestConfigConst
======================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nest_config.h>
	
	struct NestConfigConst {
		// fields
	
		static constexpr char :target:`EngineType<doxid-struct_nest_config_const_1afb211b7006d5dbd2758c7da59672f320>`[] = "nest_json";
		static constexpr char :target:`EngineSchema<doxid-struct_nest_config_const_1a667db0d13c47f386bcbff4e54c859caf>`[] = "https://neurorobotics.net/engines/:ref:`engines_nest.json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`#/engine_nest_json";
		static constexpr std::string_view :ref:`NestPythonPath<doxid-struct_nest_config_const_1af91dbf85df08c818a7fd2f2e5be53c76>` = "PYTHONPATH=" NRP_PYNEST_PATH ":$PYTHONPATH";
		static constexpr std::string_view :ref:`NestExecutablePath<doxid-struct_nest_config_const_1a5746c7343ceeac47391bcd345fec0951>` = "PATH=$PATH:" NRP_NEST_BIN_PATH;
		static constexpr std::string_view :ref:`NestRNGSeedArg<doxid-struct_nest_config_const_1a7155372f60e46303bdc070724082b9e3>` = "--nrprng";
		static constexpr std::string_view :ref:`InitFileExecStatus<doxid-struct_nest_config_const_1a4a71f164bdc05e9ce0cdb5cc8992c2eb>` = "InitExecStatus";
		static constexpr std::string_view :ref:`ResetExecStatus<doxid-struct_nest_config_const_1a31cec1e07cd58a8c425ae0602647a723>` = "ResetStatus";
		static constexpr std::string_view :ref:`InitFileParseDevMap<doxid-struct_nest_config_const_1a91dc89eeff3bf8f3b2f2e202091b8321>` = "InitFileParseDevMap";
		static constexpr std::string_view :ref:`ErrorMsg<doxid-struct_nest_config_const_1a1854d0716eacf29148873f4fcf39cbb8>` = "Message";
	};
.. _details-struct_nest_config_const:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. index:: pair: variable; NestPythonPath
.. _doxid-struct_nest_config_const_1af91dbf85df08c818a7fd2f2e5be53c76:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view NestPythonPath = "PYTHONPATH=" NRP_PYNEST_PATH ":$PYTHONPATH"

Python Path to Nest. Automatically generated via cmake on installation.

.. index:: pair: variable; NestExecutablePath
.. _doxid-struct_nest_config_const_1a5746c7343ceeac47391bcd345fec0951:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view NestExecutablePath = "PATH=$PATH:" NRP_NEST_BIN_PATH

Path to NRP Nest Server Executable. Automatically generated via cmake on installation.

.. index:: pair: variable; NestRNGSeedArg
.. _doxid-struct_nest_config_const_1a7155372f60e46303bdc070724082b9e3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view NestRNGSeedArg = "--nrprng"

Argument to pass RNG seed argument to Nest.

.. index:: pair: variable; InitFileExecStatus
.. _doxid-struct_nest_config_const_1a4a71f164bdc05e9ce0cdb5cc8992c2eb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view InitFileExecStatus = "InitExecStatus"

After the server executes the init file, this status flag will either be 1 for success or 0 for fail. If the execution fails, a JSON message with more details will be passed as well (under ErrorMsg).

.. index:: pair: variable; ResetExecStatus
.. _doxid-struct_nest_config_const_1a31cec1e07cd58a8c425ae0602647a723:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view ResetExecStatus = "ResetStatus"

After the server executes the init file, this status flag will either be 1 for success or 0 for fail. If the execution fails, a JSON message with more details will be passed as well (under ErrorMsg).

.. index:: pair: variable; InitFileParseDevMap
.. _doxid-struct_nest_config_const_1a91dc89eeff3bf8f3b2f2e202091b8321:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view InitFileParseDevMap = "InitFileParseDevMap"

After the server executes the init file, the parsed devMap will be passed back with this param.

.. index:: pair: variable; ErrorMsg
.. _doxid-struct_nest_config_const_1a1854d0716eacf29148873f4fcf39cbb8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view ErrorMsg = "Message"

If the init file could not be parsed, the python error message will be stored under this JSON property name.

