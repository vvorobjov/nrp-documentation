.. index:: pair: page; Explanation of Engine CMakeLists.txt
.. _doxid-tutorial_engine_creation_engine_cmake_example_explanation:

Explanation of Engine CMakeLists.txt
====================================

Below is described the basic structure expected in an Engine cmake configuration file. The code samples are taken from *docs/example_engine/CMakeLists.txt*, which can be used as a template.

Create basic variable definitions. These will be used in the code later on

.. ref-code-block:: cpp

	set(PROJECT_NAME "NRPExampleEngine")
	set(HEADER_DIRECTORY "nrp_example_engine")
	
	set(NAMESPACE_NAME "${PROJECT_NAME}")
	
	set(LIBRARY_NAME "${PROJECT_NAME}")
	set(PYTHON_MODULE_NAME "example_engine")
	set(EXECUTABLE_NAME "NRPExampleServerExecutable")
	set(TEST_NAME "${PROJECT_NAME}Tests")
	
	set(LIB_EXPORT_NAME "${LIBRARY_NAME}Targets")
	set(LIB_CONFIG_NAME "${LIBRARY_NAME}Config")
	set(LIB_VERSION_NAME "${LIB_CONFIG_NAME}Version")

List Cpp compile files. LIB_SRC_FILES should contain files required by the new :ref:`EngineClient <doxid-class_engine_client>` and Engine Server, PYTHON_MODULE_SRC_FILES should contain files required for integrating datapacks into TransceiverFunctions, and EXEC_SRC_FILES should contain files required by the forked Engine Server process, in particular the source file containing the :ref:`main() <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>` function.

.. ref-code-block:: cpp

	# List library build files
	set(LIB_SRC_FILES
	    nrp_example_engine/engine_server/example_engine_server.cpp
	    nrp_example_engine/nrp_client/example_engine_client.cpp
	)
	
	# List of Python module build files
	set(PYTHON_MODULE_SRC_FILES
	    nrp_example_engine/python/example_engine_python.cpp
	)
	
	# List executable build files
	set(EXEC_SRC_FILES
	    example_engine_server_executable/:ref:`main <doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>`.cpp
	    example_engine_server_executable/example_engine_server_executable.cpp
	)
	
	# List testing build files
	set(TEST_SRC_FILES
	)

Create configuration files. These files use CMake variables to insert compile-time information into the source code, mainly things such as the install location, library names, ...

.. ref-code-block:: cpp

	## Header configuration
	
	# General Header defines
	set(NRP_EXAMPLE_EXECUTABLE ${EXECUTABLE_NAME})
	configure_file("nrp_example_engine/config/cmake_constants.h.in" "${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIRECTORY}/config/cmake_constants.h" @ONLY)
	
	# Python module dependencies
	configure_file("nrp_example_engine/python/__init__.py.in" "${CMAKE_CURRENT_BINARY_DIR}/src/__init__.py" @ONLY)

Add a library target. This instructs CMake to create a library object containing the source files defined in LIB_SRC_FILES. In addition, it links the new library to ${NRP_GEN_LIB_TARGET}, which is NRPGeneralLibrary.so, the base NRP library.

.. ref-code-block:: cpp

	## NRPExampleEngineLibrary
	add_library("${LIBRARY_NAME}" SHARED ${LIB_SRC_FILES})
	add_library(${NAMESPACE_NAME}::${LIBRARY_NAME} ALIAS ${LIBRARY_NAME})
	target_compile_options(${LIBRARY_NAME} PUBLIC $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:${NRP_COMMON_COMPILATION_FLAGS}>)
	target_compile_options(${LIBRARY_NAME} PUBLIC $<$<CXX_COMPILER_ID:GNU>:-fconcepts>)
	
	set_target_properties(${LIBRARY_NAME} PROPERTIES PREFIX "")
	
	target_link_libraries(${LIBRARY_NAME}
	    PUBLIC
	        ${NRP_GEN_LIB_TARGET}
	        NRPJSONEngineProtocol::NRPJSONEngineProtocol
	
	    PRIVATE
	)
	
	target_include_directories(${LIBRARY_NAME} BEFORE
	    PUBLIC 
	        "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
	        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
	        "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"
	
	    PRIVATE
	)

Add a Python module target. With this, a new library will be created which can be used as a Python module. The proceeding install code will install the new module at the correct location, so that it can be accessed by :ref:`TransceiverFunctions <doxid-class_transceiver_function>`.

.. ref-code-block:: cpp

	## example_engine
	if(NOT ${PYTHON_MODULE_SRC_FILES} STREQUAL "")
	    add_library(${PYTHON_MODULE_NAME} SHARED ${PYTHON_MODULE_SRC_FILES})
	    add_library(${NAMESPACE_NAME}::${PYTHON_MODULE_NAME} ALIAS ${PYTHON_MODULE_NAME})
	    target_compile_options(${PYTHON_MODULE_NAME} PRIVATE $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:${NRP_COMMON_COMPILATION_FLAGS}>)
	    set_target_properties(${PYTHON_MODULE_NAME} PROPERTIES PREFIX "")
	
	    target_include_directories(${PYTHON_MODULE_NAME}
	        PUBLIC
	    )
	
	    target_link_libraries(${PYTHON_MODULE_NAME}
	        PUBLIC
	            ${NAMESPACE_NAME}::${LIBRARY_NAME}
	    )
	endif()

Add an executable target. This will compile a new executable which can be executed in a forked process to run an Engine Server along with a simulation.

.. ref-code-block:: cpp

	## NRPExampleServerExecutable
	if(NOT "${EXEC_SRC_FILES}" STREQUAL "")
	    add_executable(${EXECUTABLE_NAME} ${EXEC_SRC_FILES})
	    target_link_libraries(${EXECUTABLE_NAME} ${LIBRARY_NAME})
	endif()

Add installation instructions. After compilation, this will instruct CMake on the correct location to install header files as well as all newly generated libraries, executables, and Python modules.

.. ref-code-block:: cpp

	## Installation
	
	set(INSTALL_CONFIGDIR "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")
	
	# Install library files
	install(TARGETS
	        ${LIBRARY_NAME}
	    EXPORT
	        ${LIB_EXPORT_NAME}
	    LIBRARY DESTINATION ${NRP_PLUGIN_INSTALL_DIR}
	    ARCHIVE DESTINATION ${NRP_PLUGIN_INSTALL_DIR}
	    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
	
	    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${HEADER_DIRECTORY}
	)
	
	# Install export target
	install(EXPORT ${LIB_EXPORT_NAME}
	    DESTINATION
	        ${INSTALL_CONFIGDIR}
	    FILE
	        "${LIB_EXPORT_NAME}.cmake"
	    NAMESPACE
	        "${NAMESPACE_NAME}::"
	)
	
	# Install headers
	install(DIRECTORY "${HEADER_DIRECTORY}" "${CMAKE_CURRENT_BINARY_DIR}/include/${HEADER_DIRECTORY}"
	    DESTINATION
	        ${CMAKE_INSTALL_INCLUDEDIR}
	    FILES_MATCHING
	        PATTERN "*.h"
	        PATTERN "*.hpp"
	)
	
	# Install Python module
	if(TARGET ${PYTHON_MODULE_NAME})
	    install(TARGETS ${PYTHON_MODULE_NAME}
	        DESTINATION "${PYTHON_INSTALL_DIR_REL}/${NRP_PYTHON_MODULE_NAME}/engines/${PYTHON_MODULE_NAME}")
	
	    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/src/__init__.py"
	        DESTINATION "${PYTHON_INSTALL_DIR_REL}/${NRP_PYTHON_MODULE_NAME}/engines/${PYTHON_MODULE_NAME}")
	endif()
	
	# Install executable files
	if(TARGET ${EXECUTABLE_NAME})
	    install(TARGETS ${EXECUTABLE_NAME}
	        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
	endif()
	
	# create cmake version and config files
	include(CMakePackageConfigHelpers)
	write_basic_package_version_file(
	    "${CMAKE_CURRENT_BINARY_DIR}/${LIB_VERSION_NAME}.cmake"
	    VERSION ${PROJECT_VERSION}
	    COMPATIBILITY AnyNewerVersion
	)
	
	configure_package_config_file("${CMAKE_CURRENT_LIST_DIR}/cmake/ProjectConfig.cmake.in"
	    "${CMAKE_CURRENT_BINARY_DIR}/${LIB_CONFIG_NAME}.cmake"
	    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
	)

