.. index:: pair: struct; EngineClientInterface::CompareDevInt
.. _doxid-struct_engine_client_interface_1_1_compare_dev_int:

struct EngineClientInterface::CompareDevInt
===========================================

.. toctree::
	:hidden:

DataPackInterfaceConstSharedPtr comparison. Used for set sorting.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	struct CompareDevInt: public std::less<> {
		// methods
	
		bool :target:`operator ()<doxid-struct_engine_client_interface_1_1_compare_dev_int_1aac34282fa425617119eec650aa2909e7>` (
			const :ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`& lhs,
			const :ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`& rhs
		) const;
	
		bool :target:`operator ()<doxid-struct_engine_client_interface_1_1_compare_dev_int_1aac69ac4edfee54a8183cd1059d93cebe>` (
			const :ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`& lhs,
			const std::string& name
		) const;
	
		bool :target:`operator ()<doxid-struct_engine_client_interface_1_1_compare_dev_int_1a561807f826e47f3a73d5547b26707db5>` (
			const std::string& name,
			const :ref:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`& rhs
		) const;
	};
