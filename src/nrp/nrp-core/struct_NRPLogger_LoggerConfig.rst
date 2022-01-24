.. index:: pair: struct; NRPLogger::LoggerConfig
.. _doxid-struct_n_r_p_logger_1_1_logger_config:

struct NRPLogger::LoggerConfig
==============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Logger data struct to be saved into shared memory object. :ref:`More...<details-struct_n_r_p_logger_1_1_logger_config>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct LoggerConfig {
		// fields
	
		sem_t :ref:`sem1<doxid-struct_n_r_p_logger_1_1_logger_config_1afc5e53934d8f51ef32f2bd773fe47797>`;
		uint :ref:`fileLogLevel<doxid-struct_n_r_p_logger_1_1_logger_config_1ae59f4071eecd8dc635f2d47d5b5f6bd9>`;
		uint :ref:`consoleLogLevel<doxid-struct_n_r_p_logger_1_1_logger_config_1a04560b739713ee184b3c9e63f061bb19>`;
		size_t :ref:`logDirLen<doxid-struct_n_r_p_logger_1_1_logger_config_1ab5365f7dd0f0b45a1005aee53039440a>`;
		char :ref:`logDir<doxid-struct_n_r_p_logger_1_1_logger_config_1a6fff2631eb4e4264ca367b9a601797a7>`[1024];
	};
.. _details-struct_n_r_p_logger_1_1_logger_config:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Logger data struct to be saved into shared memory object.

Fields
------

.. index:: pair: variable; sem1
.. _doxid-struct_n_r_p_logger_1_1_logger_config_1afc5e53934d8f51ef32f2bd773fe47797:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	sem_t sem1

A semaphore indicating that the launcher has saved the settings

.. index:: pair: variable; fileLogLevel
.. _doxid-struct_n_r_p_logger_1_1_logger_config_1ae59f4071eecd8dc635f2d47d5b5f6bd9:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	uint fileLogLevel

Equivalent _fileLogLevel

.. index:: pair: variable; consoleLogLevel
.. _doxid-struct_n_r_p_logger_1_1_logger_config_1a04560b739713ee184b3c9e63f061bb19:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	uint consoleLogLevel

Equivalent _consoleLogLevel

.. index:: pair: variable; logDirLen
.. _doxid-struct_n_r_p_logger_1_1_logger_config_1ab5365f7dd0f0b45a1005aee53039440a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	size_t logDirLen

The number of symbols in _logDir

.. index:: pair: variable; logDir
.. _doxid-struct_n_r_p_logger_1_1_logger_config_1a6fff2631eb4e4264ca367b9a601797a7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	char logDir[1024]

The char buffer for storage of the _logDir

