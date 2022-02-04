.. index:: pair: namespace; proto_field_ops
.. _doxid-namespaceproto__field__ops:

namespace proto_field_ops
=========================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	namespace proto_field_ops {

	// global functions

	bpy::object :target:`GetScalarField<doxid-namespaceproto__field__ops_1a74680dd8258a713e5e29cc2dd6c4d7c7>`(gpb::Message& m, const gpb::FieldDescriptor* field);

	std::string :target:`GetScalarFieldAsString<doxid-namespaceproto__field__ops_1aa0fbc9cdff8ddaebeb8d83b70920c45c>`(
		const gpb::Message& m,
		const gpb::FieldDescriptor* field
	);

	bpy::object :target:`GetRepeatedScalarField<doxid-namespaceproto__field__ops_1acde4b42c055ec10c1dc6193354d2ba48>`(
		gpb::Message& m,
		const gpb::FieldDescriptor* field,
		int index
	);

	void :target:`SetScalarField<doxid-namespaceproto__field__ops_1a14736a7758b0ca1943e2cea845317c23>`(
		gpb::Message& m,
		const gpb::FieldDescriptor* field,
		const bpy::object& value
	);

	void :target:`SetRepeatedScalarField<doxid-namespaceproto__field__ops_1afa71d1ff5277a549d77d3ab1b22fbac6>`(
		gpb::Message& m,
		const gpb::FieldDescriptor* field,
		const bpy::object& value,
		int index
	);

	void :target:`AddRepeatedScalarField<doxid-namespaceproto__field__ops_1a9480372fc0c45dae5414bc2ad2093679>`(
		gpb::Message& m,
		const gpb::FieldDescriptor* field,
		const bpy::object& value
	);

	} // namespace proto_field_ops
