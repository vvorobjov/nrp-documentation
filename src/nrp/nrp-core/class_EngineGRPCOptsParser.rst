.. index:: pair: class; EngineGRPCOptsParser
.. _doxid-class_engine_g_r_p_c_opts_parser:

class EngineGRPCOptsParser
==========================

.. toctree::
	:hidden:

Engine GRPC Executable parameter parser.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_grpc_opts_parser.h>
	
	class EngineGRPCOptsParser {
	public:
		// methods
	
		static cxxopts::Options :target:`createOptionParser<doxid-class_engine_g_r_p_c_opts_parser_1a52fa1ab54f06440993f9842d9e7de779>`(bool allowUnrecognised = false);
	
		static cxxopts::ParseResult :target:`parseOpts<doxid-class_engine_g_r_p_c_opts_parser_1a4f1e10e5188f0db18c57887c79f35cfc>`(
			int argc,
			char* argv[],
			cxxopts::Options parser = :ref:`EngineGRPCOptsParser::createOptionParser<doxid-class_engine_g_r_p_c_opts_parser_1a52fa1ab54f06440993f9842d9e7de779>`()
		);
	};
