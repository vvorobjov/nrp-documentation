.. index:: pair: class; NRPLogger
.. _doxid-class_n_r_p_logger:

class NRPLogger
===============

.. toctree::
	:hidden:

	struct_NRPLogger_LoggerConfig.rst

Overview
~~~~~~~~

NRP Logging functions. :ref:`More...<details-class_n_r_p_logger>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nrp_logger.h>
	
	class NRPLogger {
	public:
		// typedefs
	
		typedef spdlog::level::level_enum :ref:`level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>`;
		typedef void(&)(const std::string&) :ref:`spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>`;

		// structs
	
		struct :ref:`LoggerConfig<doxid-struct_n_r_p_logger_1_1_logger_config>`;

		// construction
	
		:ref:`NRPLogger<doxid-class_n_r_p_logger_1a738cdba50f6421d19c648b621d473e1e>`(std::string loggerName = _defaultLoggerName.data());
	
		:ref:`NRPLogger<doxid-class_n_r_p_logger_1a0a50f88de5efa9809a472eb23b2d102a>`(
			std::string loggerName,
			:ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` fileLogLevel,
			:ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` consoleLogLevel,
			std::string logDir,
			bool doSavePars = false
		);

		// methods
	
		void :ref:`flush<doxid-class_n_r_p_logger_1a032a8f704f05d4277a61f6241d5b0d61>`();
		static std::string :ref:`level_to_string<doxid-class_n_r_p_logger_1a9d5c29367a4bc9c1b459575dec094463>`(const :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>`& level);
		static :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` :ref:`level_from_string<doxid-class_n_r_p_logger_1a80f2662c9efac91e02cd5048da26886c>`(const std::string& level);
		static void :ref:`shutdownDefault<doxid-class_n_r_p_logger_1aa891f50d5eb76e9ad13ad5229f0fd55e>`();
		static spdlog::logger& :ref:`nrpLogger<doxid-class_n_r_p_logger_1a0158a4d388a5a318373bd7bd751cf3ae>`();
	
		template <typename FormatString, typename... Args>
		static void :ref:`debug<doxid-class_n_r_p_logger_1a3569927c3a39e4f147974bf0e4e32144>`(
			const FormatString& fmt,
			const Args&... args
		);
	
		template <typename FormatString, typename... Args>
		static void :ref:`info<doxid-class_n_r_p_logger_1a678e6c965eb9789445a809d6ab3bb6df>`(
			const FormatString& fmt,
			const Args&... args
		);
	
		template <typename FormatString, typename... Args>
		static void :ref:`warn<doxid-class_n_r_p_logger_1a4ff3f2d99a1de213459e83c75a14ad74>`(
			const FormatString& fmt,
			const Args&... args
		);
	
		template <typename FormatString, typename... Args>
		static void :ref:`error<doxid-class_n_r_p_logger_1aa65a434aff8a9c1baba15395e86a2ad4>`(
			const FormatString& fmt,
			const Args&... args
		);
	
		template <typename FormatString, typename... Args>
		static void :ref:`critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`(
			const FormatString& fmt,
			const Args&... args
		);
	
		template <typename Message>
		static void :ref:`debug<doxid-class_n_r_p_logger_1aeaead5a30fbbba04898c4f4f31a7c62d>`(const Message& msg);
	
		template <typename Message>
		static void :ref:`info<doxid-class_n_r_p_logger_1abc4b54b27ee0571ed9a16b92b950d1ae>`(const Message& msg);
	
		template <typename Message>
		static void :ref:`warn<doxid-class_n_r_p_logger_1a2a81e77d8557b93a135edaf6859d0fea>`(const Message& msg);
	
		template <typename Message>
		static void :ref:`error<doxid-class_n_r_p_logger_1af669212ddcab3218252b193b0e4d2150>`(const Message& msg);
	
		template <typename Message>
		static void :ref:`critical<doxid-class_n_r_p_logger_1a7d458ad367b0df9478ae6d9a6a3cfe91>`(const Message& msg);
	};
.. _details-class_n_r_p_logger:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

NRP Logging functions.

Typedefs
--------

.. index:: pair: typedef; level_t
.. _doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	typedef spdlog::level::level_enum level_t

The wrapper type for log levels.

.. index:: pair: typedef; spdlog_out_fcn_t
.. _doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	typedef void(&)(const std::string&) spdlog_out_fcn_t

Logging function type, is used by Exception.

Construction
------------

.. index:: pair: function; NRPLogger
.. _doxid-class_n_r_p_logger_1a738cdba50f6421d19c648b621d473e1e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	NRPLogger(std::string loggerName = _defaultLoggerName.data())

The creation of the configurable instance of spdlog, that is set to default logger, with default settings.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- loggerName

		- The name of the logger, that is to be displayed in message and placed as log file prefix

.. index:: pair: function; NRPLogger
.. _doxid-class_n_r_p_logger_1a0a50f88de5efa9809a472eb23b2d102a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	NRPLogger(
		std::string loggerName,
		:ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` fileLogLevel,
		:ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` consoleLogLevel,
		std::string logDir,
		bool doSavePars = false
	)

The creation of the configurable instance of spdlog, that is set to default logger.

If the \launcher is set true, then the constructor tries to save the logger settings to the shared memory object. Otherwise, the constructor tries to load this settings from the shared memory object. In case of success, these settings are applied to the logger to be created, otherwise the provided settings are used.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- loggerName

		- The name of the logger, that is to be displayed in message and placed as log file prefix

	*
		- logDir

		- The location for the log files

	*
		- fileLogLevel

		- The minimum log level to be put to the log files

	*
		- consoleLogLevel

		- The minimum log level to be printed in console

	*
		- doSavePars

		- Save logger parameters to shared memory or not (load them instaed)

Methods
-------

.. index:: pair: function; flush
.. _doxid-class_n_r_p_logger_1a032a8f704f05d4277a61f6241d5b0d61:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void flush()

Flush default logger.

.. index:: pair: function; level_to_string
.. _doxid-class_n_r_p_logger_1a9d5c29367a4bc9c1b459575dec094463:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static std::string level_to_string(const :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>`& level)

Wrapper function for converting enumed log level into string.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- level

		- The numbered representation of the log level

.. index:: pair: function; level_from_string
.. _doxid-class_n_r_p_logger_1a80f2662c9efac91e02cd5048da26886c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static :ref:`NRPLogger::level_t<doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` level_from_string(const std::string& level)

Wrapper function for getting enumed log level from string. Non-valid string is converted to enum::off.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- level

		- The string representation of the log level

.. index:: pair: function; shutdownDefault
.. _doxid-class_n_r_p_logger_1aa891f50d5eb76e9ad13ad5229f0fd55e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static void shutdownDefault()

Shutdown default logger.

.. index:: pair: function; nrpLogger
.. _doxid-class_n_r_p_logger_1a0158a4d388a5a318373bd7bd751cf3ae:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static spdlog::logger& nrpLogger()

Get default :ref:`NRPLogger <doxid-class_n_r_p_logger>`.

.. index:: pair: function; debug
.. _doxid-class_n_r_p_logger_1a3569927c3a39e4f147974bf0e4e32144:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename FormatString, typename... Args>
	static void debug(
		const FormatString& fmt,
		const Args&... args
	)

NRP logging function with message formatting for debug level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fmt

		- Message format string in fmt library style `https://fmt.dev/latest/index.html <https://fmt.dev/latest/index.html>`__

	*
		- args

		- Arguments for substitution into format string #fmt

.. index:: pair: function; info
.. _doxid-class_n_r_p_logger_1a678e6c965eb9789445a809d6ab3bb6df:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename FormatString, typename... Args>
	static void info(
		const FormatString& fmt,
		const Args&... args
	)

NRP logging function with message formatting for info level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fmt

		- Message format string in fmt library style `https://fmt.dev/latest/index.html <https://fmt.dev/latest/index.html>`__

	*
		- args

		- Arguments for substitution into format string #fmt

.. index:: pair: function; warn
.. _doxid-class_n_r_p_logger_1a4ff3f2d99a1de213459e83c75a14ad74:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename FormatString, typename... Args>
	static void warn(
		const FormatString& fmt,
		const Args&... args
	)

NRP logging function with message formatting for warning level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fmt

		- Message format string in fmt library style `https://fmt.dev/latest/index.html <https://fmt.dev/latest/index.html>`__

	*
		- args

		- Arguments for substitution into format string #fmt

.. index:: pair: function; error
.. _doxid-class_n_r_p_logger_1aa65a434aff8a9c1baba15395e86a2ad4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename FormatString, typename... Args>
	static void error(
		const FormatString& fmt,
		const Args&... args
	)

NRP logging function with message formatting for error level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fmt

		- Message format string in fmt library style `https://fmt.dev/latest/index.html <https://fmt.dev/latest/index.html>`__

	*
		- args

		- Arguments for substitution into format string #fmt

.. index:: pair: function; critical
.. _doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename FormatString, typename... Args>
	static void critical(
		const FormatString& fmt,
		const Args&... args
	)

NRP logging function with message formatting for critical error level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- fmt

		- Message format string in fmt library style `https://fmt.dev/latest/index.html <https://fmt.dev/latest/index.html>`__

	*
		- args

		- Arguments for substitution into format string #fmt

.. index:: pair: function; debug
.. _doxid-class_n_r_p_logger_1aeaead5a30fbbba04898c4f4f31a7c62d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename Message>
	static void debug(const Message& msg)

NRP logging function for debug level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- msg

		- The message to be logged

.. index:: pair: function; info
.. _doxid-class_n_r_p_logger_1abc4b54b27ee0571ed9a16b92b950d1ae:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename Message>
	static void info(const Message& msg)

NRP logging function for info level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- msg

		- The message to be logged

.. index:: pair: function; warn
.. _doxid-class_n_r_p_logger_1a2a81e77d8557b93a135edaf6859d0fea:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename Message>
	static void warn(const Message& msg)

NRP logging function for warning level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- msg

		- The message to be logged

.. index:: pair: function; error
.. _doxid-class_n_r_p_logger_1af669212ddcab3218252b193b0e4d2150:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename Message>
	static void error(const Message& msg)

NRP logging function for error level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- msg

		- The message to be logged

.. index:: pair: function; critical
.. _doxid-class_n_r_p_logger_1a7d458ad367b0df9478ae6d9a6a3cfe91:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename Message>
	static void critical(const Message& msg)

NRP logging function for critical error level.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- msg

		- The message to be logged

