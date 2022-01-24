.. index:: pair: class; OutputDummy
.. _doxid-class_output_dummy:

class OutputDummy
=================

.. toctree::
	:hidden:

Dummy output node which just stores the last received msgs.


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <output_dummy.h>
	
	class OutputDummy: public :ref:`OutputNode<doxid-class_output_node>` {
	public:
		// fields
	
		size_t :target:`call_count<doxid-class_output_dummy_1adca3e3cfbacbc8e1e94855de797d8f7d>` = 0;
		const boost::python::object* :target:`lastData<doxid-class_output_dummy_1a7341f5062f91fbd9912a01a17b91cd43>` = nullptr;
		bool :target:`debugPrint<doxid-class_output_dummy_1a14fd6bff02725c7f9ec6d4e43556c8a4>` = false;

		// construction
	
		:target:`OutputDummy<doxid-class_output_dummy_1af9ea5c3f9d0f687604bfb9216a176cb0>`(const std::string& id);
	};

Inherited Members
-----------------

.. ref-code-block:: cpp
	:class: doxyrest-overview-inherited-code-block

	public:
		// enums
	
		enum :ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>`;

		// methods
	
		bool :ref:`isVisited<doxid-class_computational_node_1aa657870e632df5c4ed11470d10e65d5f>`() const;
		void :ref:`setVisited<doxid-class_computational_node_1a4d75c538b8bbc2f9c26278d27ad8a434>`(bool visited);
		const std::string& :ref:`id<doxid-class_computational_node_1aeab0953471cf02647c0264c8b474afb5>`() const;
		:ref:`NodeType<doxid-class_computational_node_1a6af2021042070fa763b8f2a0d879a4c0>` :ref:`type<doxid-class_computational_node_1a4cb10cde56ec02dd5e31c5b5498388ed>`() const;
		virtual void :ref:`configure<doxid-class_computational_node_1abf931072caba0154df7a388c515b43d8>`() = 0;
		virtual void :ref:`compute<doxid-class_computational_node_1a593ccb6e371475e628e11253b562a7a2>`() = 0;
		virtual void :ref:`configure<doxid-class_output_node_1ab38541a1582f99a453254c5ebec2449e>`();
		virtual void :ref:`compute<doxid-class_output_node_1a8bc64d2d817505fbfd68cc266ec7a9f0>`();
	
		template <class T_IN>
		:ref:`InputPort<doxid-class_input_port>`<T_IN, DATA>* :ref:`getOrRegisterInput<doxid-class_output_node_1a833d82862c1d613aab1448c3e0157237>`(const std::string& id);
	
		:ref:`OutputNodePolicies::MsgPublishPolicy<doxid-namespace_output_node_policies_1aba66d33129b6901ceb761975d08579c2>` :ref:`msgPublishPolicy<doxid-class_output_node_1a76f8b117583989b59d210f3c4759d765>`();

