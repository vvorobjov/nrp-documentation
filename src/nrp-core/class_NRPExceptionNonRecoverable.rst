.. index:: pair: class; NRPExceptionNonRecoverable
.. _doxid-class_n_r_p_exception_non_recoverable:

class NRPExceptionNonRecoverable
================================

.. toctree::
	:hidden:

Exception for non-recoverable errors.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nrp_exceptions.h>
	
	class NRPExceptionNonRecoverable: public :ref:`NRPException<doxid-class_n_r_p_exception>` {
	public:
		// construction
	
		template <class T>
		:target:`NRPExceptionNonRecoverable<doxid-class_n_r_p_exception_non_recoverable_1af0e258d45c8824dc4f15a59e53ce4ffe>`(T&& msg, bool msgLogged = false);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// methods
	
		template <class EXCEPTION>
		static void :ref:`logOnce<doxid-class_n_r_p_exception_1a846445e0e3280afb6d55a7dd9abfaa02>`(
			EXCEPTION& exception,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall = :ref:`NRPLogger::critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`
		);
	
		template <class EXCEPTION = NRPExceptionNonRecoverable, class LOG_EXCEPTION_T>
		static EXCEPTION :ref:`logCreate<doxid-class_n_r_p_exception_1a2935667821e7fa7b1605625593b1e8cb>`(
			LOG_EXCEPTION_T& exception,
			const std::string& msg,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall = :ref:`NRPLogger::critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`
		);
	
		template <class EXCEPTION = NRPExceptionNonRecoverable>
		static EXCEPTION :ref:`logCreate<doxid-class_n_r_p_exception_1abe5487fcfaf3f125f3c43446890c4915>`(
			const std::string& msg,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall = :ref:`NRPLogger::critical<doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`
		);
	
		static void :ref:`logCreate<doxid-class_n_r_p_exception_1ad287e0741ca0685566d0f550002f9031>`(
			const std::string& msg,
			:ref:`NRPLogger::spdlog_out_fcn_t<doxid-class_n_r_p_logger_1a62761d98810df56e15a777b60f8369dc>` spdlogCall
		);
	
		const char* :ref:`what<doxid-class_n_r_p_exception_1aeae52ee738c57b2c0e0ae14c6e1c42ad>`() const;

