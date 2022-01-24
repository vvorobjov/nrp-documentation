.. index:: pair: namespace; json_converter
.. _doxid-namespacejson__converter:

namespace json_converter
========================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	namespace json_converter {

	// global functions

	static PyObject* :target:`convertJsonToPyDict<doxid-namespacejson__converter_1ada67cba2a5c989f950088c9d4895686a>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json);
	static PyObject* :target:`convertJsonToPyList<doxid-namespacejson__converter_1a6875727ea29eb5924619f79e7da359d7>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json);
	static void :target:`generatePythonError<doxid-namespacejson__converter_1a4c0d2144bd8de0f564015cde3054af1b>`(const std::string& errorMessage);
	PyObject* :ref:`convertJsonToPyObject<doxid-namespacejson__converter_1a4b3825153dc600863e282a4e2a689503>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json);

	template <typename VECTOR_TYPE>
	static :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :target:`convertNumpyArrayHelper<doxid-namespacejson__converter_1afb0a3b1735b61407beaa40e9441bc5b0>`(const boost::python::numpy::ndarray& npArray);

	template <typename BUILTIN_TYPE>
	static bool :target:`compareDtype<doxid-namespacejson__converter_1ac463f4f1bcfc0f11d0e79a7ad7dae003>`(np::dtype dtype);

	:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :target:`convertNumpyArrayToJson<doxid-namespacejson__converter_1abaf4e419571b89d267f97841b93440d8>`(PyObject* value);
	static :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :target:`convertPyListToJson<doxid-namespacejson__converter_1ae85a9ebfba17a26b4dabb562cc57187e>`(PyObject* value);
	static :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :target:`convertPyTupleToJson<doxid-namespacejson__converter_1a7fdf1d1d153a22ef524ddb0beb2a46e3>`(PyObject* value);
	static :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :target:`convertPyDictToJson<doxid-namespacejson__converter_1a01592d875f09d6fdff4ca7954e5cb56a>`(PyObject* value);
	static bool :target:`isNumpyInitialized<doxid-namespacejson__converter_1acf38235b98f9d7c26822ee750b7e80e9>`();
	:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` :ref:`convertPyObjectToJson<doxid-namespacejson__converter_1a428c7fca41e33bcd78aa478cf0884167>`(PyObject* value);
	void :ref:`initNumpy<doxid-namespacejson__converter_1a7ee4665219384a54a0c12c47d24117a8>`();

	} // namespace json_converter
.. _details-namespacejson__converter:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Global Functions
----------------

.. index:: pair: function; convertJsonToPyObject
.. _doxid-namespacejson__converter_1a4b3825153dc600863e282a4e2a689503:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	PyObject* convertJsonToPyObject(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json)

Converts given JSON object into a python object.

.. index:: pair: function; convertPyObjectToJson
.. _doxid-namespacejson__converter_1a428c7fca41e33bcd78aa478cf0884167:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>` convertPyObjectToJson(PyObject* value)

Converts given python object into a JSON object.

.. index:: pair: function; initNumpy
.. _doxid-namespacejson__converter_1a7ee4665219384a54a0c12c47d24117a8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void initNumpy()

Initializes numpy array API for this module.

The function must be called before using the module. If it's not done, some of the other functions in the module may fail with an exception. The function must be called after Py_Initialize().

