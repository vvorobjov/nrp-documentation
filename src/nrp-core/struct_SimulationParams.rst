.. index:: pair: struct; SimulationParams
.. _doxid-struct_simulation_params:

struct SimulationParams
=======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

NRP Simulation Startup Parameters. :ref:`More...<details-struct_simulation_params>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <simulation_manager.h>
	
	struct SimulationParams {
		// typedefs
	
		typedef bool :target:`ParamHelpT<doxid-struct_simulation_params_1a98eade2137587ec773c048f50fd9f5e2>`;
		typedef std::string :target:`ParamSimCfgFileT<doxid-struct_simulation_params_1ada4c4d14ad6f7a6c53daa395a98b01db>`;
		typedef std::vector<std::string> :target:`ParamPluginsT<doxid-struct_simulation_params_1ab7cddd90535bd0de8471dfcd19922590>`;
		typedef std::string :target:`ParamExpDirT<doxid-struct_simulation_params_1a556afe1ead653ec292e505173aa5d4f5>`;
		typedef std::string :target:`ParamConsoleLogLevelT<doxid-struct_simulation_params_1a33c2767e0615b435d3e577399e7fa7c6>`;
		typedef std::string :target:`ParamFileLogLevelT<doxid-struct_simulation_params_1a2a81295be82f0599af9dbd686fa58301>`;
		typedef std::string :target:`ParamLogDirT<doxid-struct_simulation_params_1aa6655eec4c3816de925463de6f9d29b7>`;
		typedef std::string :target:`ParamModeT<doxid-struct_simulation_params_1abcad1c99ad6c96dd4acd464d74170a18>`;
		typedef std::string :target:`ParamServerAddressT<doxid-struct_simulation_params_1a33841425df038a18ba057e74df43cf47>`;

		// fields
	
		static constexpr std::string_view :target:`NRPProgramName<doxid-struct_simulation_params_1a05916ef3a7e8d8d73d9ff2ba66dcb1e2>` = "NRPCoreSim";
		static constexpr std::string_view :target:`ProgramDescription<doxid-struct_simulation_params_1aab5cb8ec04b7258de928ff1a0796300e>` = "Brain and physics simulator";
		static constexpr std::string_view :target:`ParamHelp<doxid-struct_simulation_params_1a2e7cbda85ae9f5cb11a705028b2f94ef>` = "h";
		static constexpr std::string_view :target:`ParamHelpLong<doxid-struct_simulation_params_1a9f563937c1d281f836efd7bec27b39ea>` = "h,help";
		static constexpr std::string_view :target:`ParamHelpDesc<doxid-struct_simulation_params_1ae518faed0097933660cec54c5cd3ab30>` = "Print this message";
		static constexpr std::string_view :target:`ParamSimCfgFile<doxid-struct_simulation_params_1ae62c32bb912e42f5f16689fe75046c85>` = "c";
		static constexpr std::string_view :target:`ParamSimCfgFileLong<doxid-struct_simulation_params_1a9fac3c8c3b307fdfd1da51e11c651a83>` = "c,config";
		static constexpr std::string_view :target:`ParamSimCfgFileDesc<doxid-struct_simulation_params_1a7f7fbd66d82dd25317cbad87148bc57d>` = "Simulation config file";
		static constexpr std::string_view :target:`ParamPlugins<doxid-struct_simulation_params_1a23c6e739580f8789d98ec6bcf136f3b6>` = "p";
		static constexpr std::string_view :target:`ParamPluginsLong<doxid-struct_simulation_params_1a659f6e701a633d24ecf2c1f70236bd59>` = "p,plugins";
		static constexpr std::string_view :target:`ParamPluginsDesc<doxid-struct_simulation_params_1aaed74c75e28e82785f0858279d8a53c3>` = "Additional engine plugins to load";
		static constexpr std::string_view :target:`ParamExpDir<doxid-struct_simulation_params_1aa7715dcf215007f249f09ca0d0461439>` = "d";
		static constexpr std::string_view :target:`ParamExpDirLong<doxid-struct_simulation_params_1a6ab636f9fd4fb875f996336835855fc0>` = "d,dir";
		static constexpr std::string_view :target:`ParamExpDirDesc<doxid-struct_simulation_params_1a4f97379be4add9bfda915ac3daba4180>` = "The explicit location of the experiment folder";
		static constexpr std::string_view :target:`ParamConsoleLogLevelLong<doxid-struct_simulation_params_1ad0c210f3627ea6ddabef6b87cce146e3>` = "cloglevel";
		static constexpr std::string_view :target:`ParamConsoleLogLevelDesc<doxid-struct_simulation_params_1a133584c296043735cd0ac85e41eec3c5>` = "Console minimum level of log severity(info by default)";
		static constexpr std::string_view :target:`ParamFileLogLevelLong<doxid-struct_simulation_params_1a0f3b671c839acb07009c0a8364bb6ff0>` = "floglevel";
		static constexpr std::string_view :target:`ParamFileLogLevelDesc<doxid-struct_simulation_params_1a2fefabf3af147d3496efadd136c86e87>` = "File minimum level of log severity(off by default)";
		static constexpr std::string_view :target:`ParamLogDirLong<doxid-struct_simulation_params_1a8cd6baa4e88c1d085696ac5a0b70aaf7>` = "logdir";
		static constexpr std::string_view :target:`ParamLogDirDesc<doxid-struct_simulation_params_1a622dfe3e7705b24bfdf952cba3a76726>` = "Directory for the file logs";
		static constexpr std::string_view :target:`ParamMode<doxid-struct_simulation_params_1ace9e8908d4634abf3e96b43ca86f5683>` = "m";
		static constexpr std::string_view :target:`ParamModeLong<doxid-struct_simulation_params_1a804ecbc40452fa7f8a2bf024228bdf8e>` = "m,mode";
		static constexpr std::string_view :target:`ParamModeDesc<doxid-struct_simulation_params_1a6a8d59e9f9b6cd2ef585b5b6fad4a7f0>` = "Operational mode, standalone or server";
		static constexpr std::string_view :target:`ParamServerAddressLong<doxid-struct_simulation_params_1a4c1cca0c06cf34df6b1eace97f38929e>` = "server_address";
		static constexpr std::string_view :target:`ParamServerAddressDesc<doxid-struct_simulation_params_1a02633bf63585e085b6f45a7e309a30e3>` = "Desired address of the server in server operational mode";
		static const :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` :ref:`_defaultLogLevel<doxid-struct_simulation_params_1a00c5bc8654b23d2d781bb08b50204a84>` = NRPLogger::level_t::info;

		// methods
	
		static cxxopts::Options :ref:`createStartParamParser<doxid-struct_simulation_params_1aa6c9b4b4d53513b7250dc1b4b466b4b8>`();
		static :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`parseJSONFile<doxid-struct_simulation_params_1af4cb8fa8b4d8c5cb6844d8f8fbcc8e8d>`(const std::string& fileName);
		static :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` :ref:`parseLogLevel<doxid-struct_simulation_params_1a8c90775f6cc1f12bee42135e262c30e7>`(const std::string& logLevel);
	};
.. _details-struct_simulation_params:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

NRP Simulation Startup Parameters.

Fields
------

.. index:: pair: variable; _defaultLogLevel
.. _doxid-struct_simulation_params_1a00c5bc8654b23d2d781bb08b50204a84:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static const :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` _defaultLogLevel = NRPLogger::level_t::info

default log level for the acses when the parameters are specified wrongly

Methods
-------

.. index:: pair: function; createStartParamParser
.. _doxid-struct_simulation_params_1aa6c9b4b4d53513b7250dc1b4b466b4b8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static cxxopts::Options createStartParamParser()

Create a parser for start parameters.



.. rubric:: Returns:

Returns parser

.. index:: pair: function; parseJSONFile
.. _doxid-struct_simulation_params_1af4cb8fa8b4d8c5cb6844d8f8fbcc8e8d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` parseJSONFile(const std::string& fileName)

Parse a JSON File and return it's values.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fileName

		- File Name



.. rubric:: Returns:

Returns parsed JSON

.. index:: pair: function; parseLogLevel
.. _doxid-struct_simulation_params_1a8c90775f6cc1f12bee42135e262c30e7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` parseLogLevel(const std::string& logLevel)

parsing input parameter string log level into enum type



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- logLevel

		- The string expression of the log level

