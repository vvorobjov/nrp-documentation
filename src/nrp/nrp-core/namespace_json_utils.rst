.. index:: pair: namespace; json_utils
.. _doxid-namespacejson__utils:

namespace json_utils
====================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	namespace json_utils {

	// global functions

	static void :ref:`json_schema_loader<doxid-namespacejson__utils_1a58693040275e0ca8b1f1bdde261d9241>`(const json_uri& uri, :ref:`json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& schema);

	template <typename VALUE_T>
	static void :ref:`set_default<doxid-namespacejson__utils_1a70842d353dc2b137d575be7c7dbec521>`(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& instance,
		const std::string key,
		const VALUE_T new_value
	);

	static void :ref:`validate_json<doxid-namespacejson__utils_1ab4c4dcbfcffbaed1f45abc1546a27086>`(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& instance,
		std::string schema_path,
		bool addPatch = true
	);

	} // namespace json_utils
.. _details-namespacejson__utils:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Global Functions
----------------

.. index:: pair: function; json_schema_loader
.. _doxid-namespacejson__utils_1a58693040275e0ca8b1f1bdde261d9241:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static void json_schema_loader(const json_uri& uri, :ref:`json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& schema)

Loads schema from json file given its uri. To be passed to a nlohmann::json_schema::json_validator constructor.

The function is parametrized with two parameters: uri and schema. uri provides the path to the json schema within NRP_CONFIG_INSTALL_DIR directory. the schema will be loaded into "schema" json object.

.. index:: pair: function; set_default
.. _doxid-namespacejson__utils_1a70842d353dc2b137d575be7c7dbec521:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <typename VALUE_T>
	static void set_default(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& instance,
		const std::string key,
		const VALUE_T new_value
	)

Sets default value for a given parameter.

The function is parametrized with three parameters: instance, key, new_value. A value 'new_value' for parameter 'key' is set in 'instance'. If 'key' already exists in 'instance' nothing is done

.. index:: pair: function; validate_json
.. _doxid-namespacejson__utils_1ab4c4dcbfcffbaed1f45abc1546a27086:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static void validate_json(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& instance,
		std::string schema_path,
		bool addPatch = true
	)

Validates a json object using a given json schema.

Parameters: instance: json object to be validated schema_path: URI of the schema to use for validation addPatch: boolean attribute. If true, parameter default values defined in the schema are added to 'instance' after validation.

