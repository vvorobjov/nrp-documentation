.. index:: pair: page; General developer guide
.. _doxid-tutorial_developer_guide:

General developer guide
=======================

This page is supposed to give an overview of the development process set up for the NRP Core repository.



.. _doxid-tutorial_developer_guide_1tutorial_developer_guide_testing:

Testing
~~~~~~~

The testing framework of our choice is `Google Test <https://github.com/google/googletest>`__. To execute our tests we use `ctest <https://cmake.org/cmake/help/latest/manual/ctest.1.html>`__, which is a test runner provided by CMake.

*Please note that all commands in this section should be executed from the build directory!*

By default, building of the tests is enabled. To disable it, add ``-DENABLE_TESTING=OFF`` to the cmake step of the build process.

The simplest way to run all available tests, after the build and installation process is complete:

.. ref-code-block:: cpp

	make test

or, equivalently:

.. ref-code-block:: cpp

	ctest

To run a single test or a group of tests you can use the ctest regex (-R) option (-VV runs the tests in verbose mode):

.. ref-code-block:: cpp

	ctest -VV -R Simulation LoopTest.RunLoop  # Run a single test from the Simulation Loop group
	ctest -VV -R Simulation LoopTest          # Run all tests from the Simulation Loop group

Every NRP Core module should have its own tests compiled into a single executable. The executable should be named ``${PROJECT_NAME}Tests``.

To find all test executables:

.. ref-code-block:: cpp

	find . -name "*Tests"

which should give an output similar to this:

.. ref-code-block:: cpp

	./nrp_engine_protocols/nrp_grpc_engine_protocol/NRPGRPCEngineProtocolTests
	./nrp_engine_protocols/nrp_json_engine_protocol/NRPJSONEngineProtocolTests
	./nrp_general_library/NRPGeneralLibraryTests
	./nrp_gazebo_engines/nrp_gazebo_grpc_engine/NRPGazeboGrpcEngineTests
	./nrp_gazebo_engines/nrp_gazebo_json_engine/NRPGazeboJSONEngineTests
	./nrp_simulation/NRPCoreSimTests
	./nrp_python_json_engine/NRPPythonJSONEngineTests
	./nrp_nest_engines/nrp_nest_json_engine/NRPNestJSONEngineTests

To run a single executable, which contains all tests for the module:

.. ref-code-block:: cpp

	./nrp_general_library/NRPGeneralLibraryTests

To run a single test you can use the gtest filtering capability:

.. ref-code-block:: cpp

	nrp_general_library/NRPGeneralLibraryTests --gtest_filter=InterpreterTest.TestTransceiverFcnDataPacks





.. _doxid-tutorial_developer_guide_1tutorial_developer_guide_loggingg:

Logger usage
~~~~~~~~~~~~

For logging we use the own wrapper for the fast thread-safe logger SpdLog.

In order to enable logging functionality, add to the code:

.. ref-code-block:: cpp

	#include "nrp_general_library/utils/nrp_logger.h"

The logger has the following calls for printing the logs of corresponding severity:

.. ref-code-block:: cpp

	:ref:`NRPLogger::debug <doxid-class_n_r_p_logger_1a3569927c3a39e4f147974bf0e4e32144>`("debug message");
	:ref:`NRPLogger::debug <doxid-class_n_r_p_logger_1a3569927c3a39e4f147974bf0e4e32144>`("formatted string debug message {}", "Hello world!");
	:ref:`NRPLogger::info <doxid-class_n_r_p_logger_1a678e6c965eb9789445a809d6ab3bb6df>`("info message");
	:ref:`NRPLogger::info <doxid-class_n_r_p_logger_1a678e6c965eb9789445a809d6ab3bb6df>`("formatted decimal info message {0:d}", 22);
	:ref:`NRPLogger::warn <doxid-class_n_r_p_logger_1a4ff3f2d99a1de213459e83c75a14ad74>`("warn message");
	:ref:`NRPLogger::warn <doxid-class_n_r_p_logger_1a4ff3f2d99a1de213459e83c75a14ad74>`("formatted binary warn message {0:b}", 42);
	:ref:`NRPLogger::error <doxid-class_n_r_p_logger_1aa65a434aff8a9c1baba15395e86a2ad4>`("error message");
	:ref:`NRPLogger::error <doxid-class_n_r_p_logger_1aa65a434aff8a9c1baba15395e86a2ad4>`("formatted float error message {:03.2f}", 3.14);
	:ref:`NRPLogger::critical <doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`("critical message");
	:ref:`NRPLogger::critical <doxid-class_n_r_p_logger_1a04652d9ed341d88ab748341fe39bf42e>`("{:>30}", "right aligned critical message");

and, separately, trace level

.. ref-code-block:: cpp

	:ref:`NRP_LOGGER_TRACE <doxid-nrp__logger_8h_1a44d0ffe46e0db421ac193bb0eaa6e0f5>`("trace message");
	:ref:`NRP_LOGGER_TRACE <doxid-nrp__logger_8h_1a44d0ffe46e0db421ac193bb0eaa6e0f5>`("formatted string trace message {}", __FUNCTION__);

The macro NRP_LOGGER_TRACE can be totally voided if PRODUCTION_RELEASE is defined at compilation. This allows hiding all trace log calls (created by NRP_LOGGER_TRACE) from the compiled code.

Each logger can be initialized with explicitly or default parameters. This behaviour is determined by the :ref:`NRPLogger <doxid-class_n_r_p_logger>` constructor that is called. The settings can be defined explicitly in the following constructor:

.. ref-code-block:: cpp

	:ref:`NRPLogger <doxid-class_n_r_p_logger>`(
	    std::string loggerName,
	    :ref:`NRPLogger::level_t <doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` fileLogLevel,
	    :ref:`NRPLogger::level_t <doxid-class_n_r_p_logger_1a13becfae8f98ba5d14c86a101344a4b1>` consoleLogLevel,
	    std::string logDir,
	    bool doSavePars = false);

The name of the logger, ``loggerName``, is displayed in the log message and is appended to the log file name. The corresponding minimum log levels can be set for both file and console (``fileLogLevel`` and ``consoleLogLevel``). The parameter ``logDir`` specifies the location of the log files with respect to the working directory (or may be set as absolute path). The ``doSavePars`` flag allows this constructor to propagate the logger settings or consume them from the shared memory. In case this flag is ``true``, then the constructor saves settings into the shared memory, otherwise the constructor tries to load them.

The creation of the logger with the default parameters can be done with another constructor:

.. ref-code-block:: cpp

	:ref:`NRPLogger <doxid-class_n_r_p_logger>`(
	    std::string loggerName = _defaultLoggerName.data());

Even if the ``loggerName`` is not specified at the call, it will be set with the default value ``_defaultLoggerName = "nrp_core"``. The value of the other parameters is determined in the constructor definition:

.. ref-code-block:: cpp

	:ref:`NRPLogger::NRPLogger <doxid-class_n_r_p_logger_1a738cdba50f6421d19c648b621d473e1e>`(
	    std::string loggerName)
	    : :ref:`NRPLogger <doxid-class_n_r_p_logger>`(
	        loggerName, 
	        :ref:`NRPLogger <doxid-class_n_r_p_logger>`::level_t::off, 
	        :ref:`NRPLogger <doxid-class_n_r_p_logger>`::level_t::info, 
	        _defaultLogDir.data(), 
	        false) {}

Note, that using this constructor doesn't allow saving the settings of the logger (``doSavePars = false``). This constructor will always try to load them from the memory.

Currently, only the logger in the NRPCoreSim executable is initialized with explicit constructor (which is parametrized by the console parameters). And only this logger tries to save its settings to the shared memory object. The other loggers (in engine servers) try to fetch the settings from the shared memory object and apply them. In case they can’t, the default settings are applied. Thus, the child processes of the launcher inherit its logger settings by the following workflow:

#. The launcher creates the first logger and initializes it with parameters from the console (if any, or with default ones if they are absent).

#. The resulting settings from the launcher logger are saved into the shared memory

#. When the forked process starts, it creates its own logger.

#. The process tries to find the shared object with settings and get them from there

#. If something goes wrong with the shared object, the logger is initialized with the default settings (only a message is given, that it couldn’t load settings, the process is not terminated).

Here are the optional console parameters that are used to define the logger settings:

* ``-l,--loglevel <VAL>`` defines the general log level;

* ``--cloglevel <VAL>`` defines the console log level;

* ``--floglevel <VAL>`` defines the file log level;

* ``--logdir <VAL>`` defines the directory for the log files;

The values for the level parameters can be any of ``trace``, ``debug``, ``info``, ``warn``, ``error``, ``critical``.

Finally, after the :ref:`NRPLogger <doxid-class_n_r_p_logger>` object is created, it should be at some point deleted. Note, that the destructor of the :ref:`NRPLogger <doxid-class_n_r_p_logger>` closes all spdlog sinks and, thus, disables the following logging. The :ref:`NRPLogger <doxid-class_n_r_p_logger>` object should be deleted only at the end of the operation and only one :ref:`NRPLogger <doxid-class_n_r_p_logger>` should be created within the process.





.. _doxid-tutorial_developer_guide_1tutorial_developer_guide_static:

Static code analysis
~~~~~~~~~~~~~~~~~~~~

Currently we support `cppcheck <http://cppcheck.sourceforge.net/>`__ as the static code analysis tool. It is integrated into our build system. Before you can use it, you will have to install it:

.. ref-code-block:: cpp

	sudo apt install cppcheck

You can run it from the build directory with:

.. ref-code-block:: cpp

	make cppcheck





.. _doxid-tutorial_developer_guide_1tutorial_developer_guide_time_profiler:

Time Profiler
~~~~~~~~~~~~~

There are two macros available for time profiling: NRP_LOG_TIME and NRP_LOG_TIME_BLOCK. Both macros are only activated if TIME_PROFILE variable is defined at compilation. This allows easily hiding all time profile calls from the compiled code if wished.

NRP_LOG_TIME takes a ``filename`` parameter and records in a file with that name (and ``.log`` extension) the time difference, expressed in microseconds, between the clock time at the moment of calling and a fix time point. For example the next call:

.. ref-code-block:: cpp

	:ref:`NRP_LOG_TIME <doxid-time__utils_8h_1a1235b51ed357e4bcc6617c2ad8efc8b0>`("my_time_point");

will add a record with the aforementioned time difference to a file named ``my_time_point.log``.

NRP_LOG_TIME_BLOCK functions in a similar way than NRP_LOG_TIME but records the duration between the moment of calling and the end of the current block, ie. when a created helper object goes out of scope. For example:

.. ref-code-block:: cpp

	{
	    :ref:`NRP_LOG_TIME_BLOCK <doxid-time__utils_8h_1ab33b74f2735ed14b63bf2dbae0a00d12>`("my_time_duration");
	    // ... here some important code
	}

will add a record with the duration, also expressed in microseconds, until the execution reaches the end of the block to a file named my_time_duration.log.

All time log files are stored in a subfolder ``time_logs`` of the simulation working directory.

Finally, both macros are defined in ``nrp_general_library/utils/time_utils.h``, which must be included in order to use them.





.. _doxid-tutorial_developer_guide_1tutorial_developer_guide_grpc:

Debugging gRPC engines
~~~~~~~~~~~~~~~~~~~~~~

`gRPC troubleshooting guide <https://github.com/grpc/grpc/blob/master/TROUBLESHOOTING.md>`__

