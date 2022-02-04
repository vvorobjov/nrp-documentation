.. index:: pair: class; EngineJSONOptsParser
.. _doxid-class_engine_j_s_o_n_opts_parser:

class EngineJSONOptsParser
==========================

.. toctree::
	:hidden:

Engine JSON Executable parameter parser.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <engine_json_opts_parser.h>
	
	class EngineJSONOptsParser {
	public:
		// methods
	
		static cxxopts::Options :target:`createOptionParser<doxid-class_engine_j_s_o_n_opts_parser_1a824f765e22654abd91b90be47b5b7115>`(bool allowUnrecognised = false);
	
		static cxxopts::ParseResult :target:`parseOpts<doxid-class_engine_j_s_o_n_opts_parser_1a905fe31fe826b6e73b433b766414f06d>`(
			int argc,
			char* argv[],
			cxxopts::Options parser = :ref:`EngineJSONOptsParser::createOptionParser<doxid-class_engine_j_s_o_n_opts_parser_1a824f765e22654abd91b90be47b5b7115>`()
		);
	};
