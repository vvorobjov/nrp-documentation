.. index:: pair: struct; EngineGRPCConfigConst
.. _doxid-struct_engine_g_r_p_c_config_const:

struct EngineGRPCConfigConst
============================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_grpc_config.h>
	
	struct EngineGRPCConfigConst {
		// fields
	
		static constexpr short :ref:`MaxAddrBindTries<doxid-struct_engine_g_r_p_c_config_const_1ad3cbf8fab5670292a2677871c2841ac9>` = 1024;
		static constexpr std::string_view :ref:`EngineServerAddrArg<doxid-struct_engine_g_r_p_c_config_const_1ae427259e07b52ad09a8717747becf39e>` = "serverurl";
		static constexpr std::string_view :ref:`EngineNameArg<doxid-struct_engine_g_r_p_c_config_const_1a29ce71a5902dec919d685d501da85b18>` = "engine";
		static constexpr std::string_view :ref:`EngineServerGetDataPacksRoute<doxid-struct_engine_g_r_p_c_config_const_1a6fb701b241c918436e985d6ba5222688>` = "/get_datapack_information";
		static constexpr std::string_view :ref:`EngineServerSetDataPacksRoute<doxid-struct_engine_g_r_p_c_config_const_1a233a8264094529bafa59f588bfc3b598>` = "/set_datapack";
		static constexpr std::string_view :ref:`EngineServerRunLoopStepRoute<doxid-struct_engine_g_r_p_c_config_const_1a6f021c0eadc55318c39db828c347b08b>` = "/run_loop";
		static constexpr std::string_view :ref:`EngineServerInitializeRoute<doxid-struct_engine_g_r_p_c_config_const_1abc022fdce1acbeb8aa55e99b9fa41e8a>` = "/initialize";
		static constexpr std::string_view :ref:`EngineServerShutdownRoute<doxid-struct_engine_g_r_p_c_config_const_1afe6d06cc5f1b225493f82051819453e2>` = "/shutdown";
		static constexpr std::string_view :ref:`EngineTimeStepName<doxid-struct_engine_g_r_p_c_config_const_1ac66fcc7ae3fc951e7611e173c7737d38>` = "time_step";
		static constexpr std::string_view :ref:`EngineTimeName<doxid-struct_engine_g_r_p_c_config_const_1a0c0df792fe8316defa5a17afc593268b>` = "time";
		static constexpr std::string_view :ref:`EngineServerContentType<doxid-struct_engine_g_r_p_c_config_const_1a30edf756be8337ccf1c83da3c70c575e>` = "application/:ref:`json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`";
	};
.. _details-struct_engine_g_r_p_c_config_const:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Fields
------

.. index:: pair: variable; MaxAddrBindTries
.. _doxid-struct_engine_g_r_p_c_config_const_1ad3cbf8fab5670292a2677871c2841ac9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr short MaxAddrBindTries = 1024

Maximum amount of tries for EngineJSONserver to bind to different ports.

.. index:: pair: variable; EngineServerAddrArg
.. _doxid-struct_engine_g_r_p_c_config_const_1ae427259e07b52ad09a8717747becf39e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerAddrArg = "serverurl"

Parameter name that is used to pass along the server address.

.. index:: pair: variable; EngineNameArg
.. _doxid-struct_engine_g_r_p_c_config_const_1a29ce71a5902dec919d685d501da85b18:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineNameArg = "engine"

Parameter name that is used to pass along the engine name.

.. index:: pair: variable; EngineServerGetDataPacksRoute
.. _doxid-struct_engine_g_r_p_c_config_const_1a6fb701b241c918436e985d6ba5222688:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerGetDataPacksRoute = "/get_datapack_information"

REST Server Route from which to get datapack information.

.. index:: pair: variable; EngineServerSetDataPacksRoute
.. _doxid-struct_engine_g_r_p_c_config_const_1a233a8264094529bafa59f588bfc3b598:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerSetDataPacksRoute = "/set_datapack"

REST Server Route to which to send datapack changes.

.. index:: pair: variable; EngineServerRunLoopStepRoute
.. _doxid-struct_engine_g_r_p_c_config_const_1a6f021c0eadc55318c39db828c347b08b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerRunLoopStepRoute = "/run_loop"

REST Server Route to execute a single loop.

.. index:: pair: variable; EngineServerInitializeRoute
.. _doxid-struct_engine_g_r_p_c_config_const_1abc022fdce1acbeb8aa55e99b9fa41e8a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerInitializeRoute = "/initialize"

REST Server Route for engine initialization.

.. index:: pair: variable; EngineServerShutdownRoute
.. _doxid-struct_engine_g_r_p_c_config_const_1afe6d06cc5f1b225493f82051819453e2:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerShutdownRoute = "/shutdown"

REST Server Route for engine shutdown.

.. index:: pair: variable; EngineTimeStepName
.. _doxid-struct_engine_g_r_p_c_config_const_1ac66fcc7ae3fc951e7611e173c7737d38:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineTimeStepName = "time_step"

JSON name under which the runLoopStep timeStep is saved.

.. index:: pair: variable; EngineTimeName
.. _doxid-struct_engine_g_r_p_c_config_const_1a0c0df792fe8316defa5a17afc593268b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineTimeName = "time"

JSON name under which the runLoopStep engine time is sent.

.. index:: pair: variable; EngineServerContentType
.. _doxid-struct_engine_g_r_p_c_config_const_1a30edf756be8337ccf1c83da3c70c575e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static constexpr std::string_view EngineServerContentType = "application/:ref:`json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`"

Content Type passed between server and client.

