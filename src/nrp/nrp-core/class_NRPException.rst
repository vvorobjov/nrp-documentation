.. index:: pair: class; NRPException
.. _doxid-class_n_r_p_exception:

class NRPException
==================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Base :ref:`NRPException <doxid-class_n_r_p_exception>` class. :ref:`More...<details-class_n_r_p_exception>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nrp_exceptions.h>
	
	class NRPException: public exception {
	public:
		// construction
	
		template <class T>
		:target:`NRPException<doxid-class_n_r_p_exception_1a24030eadd17af020e14f5afbfe57a6d2>`(T&& msg, bool msgLogged = false);

		// methods
	
		template <class EXCEPTION>
		static void :target:`logOnce<doxid-class_n_r_p_exception_1a846445e0e3280afb6d55a7dd9abfaa02>`(
			EXCEPTION& exception,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall = :ref:`NRPLogger::critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`
		);
	
		template <class EXCEPTION = NRPExceptionNonRecoverable, class LOG_EXCEPTION_T>
		static EXCEPTION :target:`logCreate<doxid-class_n_r_p_exception_1a2935667821e7fa7b1605625593b1e8cb>`(
			LOG_EXCEPTION_T& exception,
			const std::string& msg,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall = :ref:`NRPLogger::critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`
		);
	
		template <class EXCEPTION = NRPExceptionNonRecoverable>
		static EXCEPTION :ref:`logCreate<doxid-class_n_r_p_exception_1abe5487fcfaf3f125f3c43446890c4915>`(
			const std::string& msg,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall = :ref:`NRPLogger::critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`
		);
	
		static void :target:`logCreate<doxid-class_n_r_p_exception_1ad287e0741ca0685566d0f550002f9031>`(
			const std::string& msg,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall
		);
	
		const char* :target:`what<doxid-class_n_r_p_exception_1aeae52ee738c57b2c0e0ae14c6e1c42ad>`() const;
	};

	// direct descendants

	class :ref:`NRPExceptionNonRecoverable<doxid-class_n_r_p_exception_non_recoverable>`;
	class :ref:`NRPExceptionRecoverable<doxid-class_n_r_p_exception_recoverable>`;
.. _details-class_n_r_p_exception:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Base :ref:`NRPException <doxid-class_n_r_p_exception>` class.

Methods
-------

.. index:: pair: function; logCreate
.. _doxid-class_n_r_p_exception_1abe5487fcfaf3f125f3c43446890c4915:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class EXCEPTION = NRPExceptionNonRecoverable>
	static EXCEPTION logCreate(
		const std::string& msg,
		:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall = :ref:`NRPLogger::critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`
	)

Logs the given message to the output, then returns EXCEPTION type.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- EXCEPTION

		- Exception type to return

	*
		- msg

		- Message to log and put into thrown exception

	*
		- spdlogCall

		- spdlog function to call for logging

