.. index:: pair: struct; EngineJSONConfigConst
.. _doxid-struct_engine_j_s_o_n_config_const:

struct EngineJSONConfigConst
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_json_config.h>
	
	struct EngineJSONConfigConst {
		// fields
	
		static constexpr short :ref:`MaxAddrBindTries<doxid-struct_engine_j_s_o_n_config_const_1a13f8dce8a46ede8d689107a8accd25d9>` = 1024;
		static constexpr std::string_view :ref:`EngineServerAddrArg<doxid-struct_engine_j_s_o_n_config_const_1a18382e470a6f5cacebb29969cc6c00e9>` = "serverurl";
		static constexpr std::string_view :ref:`EngineRegistrationServerAddrArg<doxid-struct_engine_j_s_o_n_config_const_1acfdf9f569454b75752973eea85410e4a>` = "regservurl";
		static constexpr std::string_view :ref:`EngineNameArg<doxid-struct_engine_j_s_o_n_config_const_1a6ccea143caf7fc04a58a98f261996a96>` = "engine";
		static constexpr std::string_view :ref:`EngineServerGetDataPacksRoute<doxid-struct_engine_j_s_o_n_config_const_1a23b4a7d46e8de94b24c324dc21b4264a>` = "/get_datapack_information";
		static constexpr std::string_view :ref:`EngineServerSetDataPacksRoute<doxid-struct_engine_j_s_o_n_config_const_1a745c135dbe5632b7a65e86117ba9fb33>` = "/set_datapack";
		static constexpr std::string_view :ref:`EngineServerRunLoopStepRoute<doxid-struct_engine_j_s_o_n_config_const_1a006965b6f8ca36b005c969026b7efa9e>` = "/run_loop";
		static constexpr std::string_view :ref:`EngineServerInitializeRoute<doxid-struct_engine_j_s_o_n_config_const_1a8380562c931822892f90e6481366dd70>` = "/initialize";
		static constexpr std::string_view :ref:`EngineServerResetRoute<doxid-struct_engine_j_s_o_n_config_const_1a44daaa35ac527173aac72648d7f71718>` = "/reset";
		static constexpr std::string_view :ref:`EngineServerShutdownRoute<doxid-struct_engine_j_s_o_n_config_const_1a419a9a308cf54ecd015172917c411595>` = "/shutdown";
		static constexpr std::string_view :ref:`EngineTimeStepName<doxid-struct_engine_j_s_o_n_config_const_1ad0245e3fde23050221e38cac0e845c95>` = "time_step";
		static constexpr std::string_view :ref:`EngineTimeName<doxid-struct_engine_j_s_o_n_config_const_1aa77ec440d3c1a35c3add1bdbe5b88a81>` = "time";
		static constexpr std::string_view :ref:`EngineServerContentType<doxid-struct_engine_j_s_o_n_config_const_1ad643fbcab0cbe6fa6b27aaa04c3f4e58>` = "application/:ref:`json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`";
	};
.. _details-struct_engine_j_s_o_n_config_const:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. index:: pair: variable; MaxAddrBindTries
.. _doxid-struct_engine_j_s_o_n_config_const_1a13f8dce8a46ede8d689107a8accd25d9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr short MaxAddrBindTries = 1024

Maximum amount of tries for EngineJSONserver to bind to different ports.

.. index:: pair: variable; EngineServerAddrArg
.. _doxid-struct_engine_j_s_o_n_config_const_1a18382e470a6f5cacebb29969cc6c00e9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerAddrArg = "serverurl"

Parameter name that is used to pass along the server address.

.. index:: pair: variable; EngineRegistrationServerAddrArg
.. _doxid-struct_engine_j_s_o_n_config_const_1acfdf9f569454b75752973eea85410e4a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineRegistrationServerAddrArg = "regservurl"

Parameter name that is used to pass along the server address.

.. index:: pair: variable; EngineNameArg
.. _doxid-struct_engine_j_s_o_n_config_const_1a6ccea143caf7fc04a58a98f261996a96:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineNameArg = "engine"

Parameter name that is used to pass along the engine name.

.. index:: pair: variable; EngineServerGetDataPacksRoute
.. _doxid-struct_engine_j_s_o_n_config_const_1a23b4a7d46e8de94b24c324dc21b4264a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerGetDataPacksRoute = "/get_datapack_information"

REST Server Route from which to get datapack information.

.. index:: pair: variable; EngineServerSetDataPacksRoute
.. _doxid-struct_engine_j_s_o_n_config_const_1a745c135dbe5632b7a65e86117ba9fb33:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerSetDataPacksRoute = "/set_datapack"

REST Server Route to which to send datapack changes.

.. index:: pair: variable; EngineServerRunLoopStepRoute
.. _doxid-struct_engine_j_s_o_n_config_const_1a006965b6f8ca36b005c969026b7efa9e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerRunLoopStepRoute = "/run_loop"

REST Server Route to execute a single loop.

.. index:: pair: variable; EngineServerInitializeRoute
.. _doxid-struct_engine_j_s_o_n_config_const_1a8380562c931822892f90e6481366dd70:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerInitializeRoute = "/initialize"

REST Server Route for engine initialization.

.. index:: pair: variable; EngineServerResetRoute
.. _doxid-struct_engine_j_s_o_n_config_const_1a44daaa35ac527173aac72648d7f71718:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerResetRoute = "/reset"

REST Server Route for engine initialization.

.. index:: pair: variable; EngineServerShutdownRoute
.. _doxid-struct_engine_j_s_o_n_config_const_1a419a9a308cf54ecd015172917c411595:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerShutdownRoute = "/shutdown"

REST Server Route for engine shutdown.

.. index:: pair: variable; EngineTimeStepName
.. _doxid-struct_engine_j_s_o_n_config_const_1ad0245e3fde23050221e38cac0e845c95:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineTimeStepName = "time_step"

JSON name under which the runLoopStep timeStep is saved.

.. index:: pair: variable; EngineTimeName
.. _doxid-struct_engine_j_s_o_n_config_const_1aa77ec440d3c1a35c3add1bdbe5b88a81:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineTimeName = "time"

JSON name under which the runLoopStep engine time is sent.

.. index:: pair: variable; EngineServerContentType
.. _doxid-struct_engine_j_s_o_n_config_const_1ad643fbcab0cbe6fa6b27aaa04c3f4e58:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerContentType = "application/:ref:`json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`"

Content Type passed between server and client.

