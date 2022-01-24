.. index:: pair: struct; TransceiverFunctionSortedResults
.. _doxid-struct_transceiver_function_sorted_results:

struct TransceiverFunctionSortedResults
=======================================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <transceiver_function_sorted_results.h>
	
	struct TransceiverFunctionSortedResults: public std::map< std::string, EngineClientInterface::datapacks_ptr_t > {
		// typedefs
	
		typedef :ref:`EngineClientInterface::datapacks_ptr_t<doxid-class_engine_client_interface_1a54f3c8965ef0b6c1ef52dbc0d9d918e8>` :target:`datapacks_t<doxid-struct_transceiver_function_sorted_results_1abe3b698c3130acbfc6e705c518dbaade>`;
		typedef std::map<std::string, :ref:`datapacks_t<doxid-struct_transceiver_function_sorted_results_1abe3b698c3130acbfc6e705c518dbaade>`> :target:`interface_results_t<doxid-struct_transceiver_function_sorted_results_1a8d2337605d7061baac77fdc50d907dcd>`;

		// methods
	
		void :ref:`addResults<doxid-struct_transceiver_function_sorted_results_1a6deb3f48f4f2936f4bc9695eb972abcb>`(const :ref:`TransceiverFunctionManager::tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>`& results);
		static TransceiverFunctionSortedResults :ref:`sortResults<doxid-struct_transceiver_function_sorted_results_1a9b52540378ae57341c5e79334efa3719>`(const :ref:`TransceiverFunctionManager::tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>`& results);
	};
.. _details-struct_transceiver_function_sorted_results:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; addResults
.. _doxid-struct_transceiver_function_sorted_results_1a6deb3f48f4f2936f4bc9695eb972abcb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void addResults(const :ref:`TransceiverFunctionManager::tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>`& results)

Add additional results to existing SortedResults.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- results

		- Results to add

.. index:: pair: function; sortResults
.. _doxid-struct_transceiver_function_sorted_results_1a9b52540378ae57341c5e79334efa3719:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static TransceiverFunctionSortedResults sortResults(const :ref:`TransceiverFunctionManager::tf_results_t<doxid-class_transceiver_function_manager_1a896a151a16f978eb6ef7d7f2e92f5ee2>`& results)

Sort results according to interface type.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- results

		- Results to sort



.. rubric:: Returns:

Returns sorted results

