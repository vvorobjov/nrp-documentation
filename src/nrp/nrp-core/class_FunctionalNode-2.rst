.. index:: pair: class; FunctionalNode<std::tuple<INPUT_TYPES...>, std::tuple<OUTPUT_TYPES...>>
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7:

template class FunctionalNode<std::tuple<INPUT_TYPES...>, std::tuple<OUTPUT_TYPES...>>
======================================================================================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Implementation of a functional node in the computational graph. :ref:`More...<details-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <functional_node.h>
	
	template <typename... INPUT_TYPES, typename... OUTPUT_TYPES>
	class FunctionalNode<std::tuple<INPUT_TYPES...>, std::tuple<OUTPUT_TYPES...>>: public :ref:`ComputationalNode<doxid-class_computational_node>` {
	public:
		// methods
	
		virtual void :ref:`configure<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a073ffcf4ae4b5bb9302e4c066f5b077c>`();
		virtual void :ref:`compute<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1acc2c359e3947b15a5c9bee5b424eab6a>`();
	
		template <std::size_t N, class T_IN, class T_OUT>
		:ref:`InputPort<doxid-class_input_port>`<T_IN, T_OUT>* :ref:`registerInput<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a9f981b908546fb816795931b70395687>`(const std::string& id);
	
		:ref:`Port<doxid-class_port>`* :ref:`getInputByIndex<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1ad7d7562762c78ab77a788a0a92c798a7>`(size_t idx);
		:ref:`Port<doxid-class_port>`* :ref:`getInputById<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1aaa0d4f4cc20813efc0887429b2363adb>`(const std::string& id);
	
		template <std::size_t N, class T>
		:ref:`OutputPort<doxid-class_output_port>`<T>* :ref:`registerOutput<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a2ad5dd10c60a903a56986bcc52dca8ed>`(const std::string& id);
	
		template <std::size_t N>
		:ref:`Port<doxid-class_port>`* :ref:`getOutputByIndex<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a3ed48bb6e11cf25d3ccd2b3e5df8084a>`();
	
		template <std::size_t N = 0>
		:ref:`Port<doxid-class_port>`* :ref:`getOutputById<doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1aaa78f26b09a1bd9c0877d35976a8bf2a>`(const std::string& id);
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

.. _details-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Implementation of a functional node in the computational graph.

It stores an std::function object, '_function' which is called in the node 'compute' method and which inputs and outputs can be connected to input and output ports respectively

Methods
-------

.. index:: pair: function; configure
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a073ffcf4ae4b5bb9302e4c066f5b077c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void configure()

Configure. Print warnings if node is not fully connected.

.. index:: pair: function; compute
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1acc2c359e3947b15a5c9bee5b424eab6a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	virtual void compute()

Compute. Execute '_function' and send its outputs out.

.. index:: pair: function; registerInput
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a9f981b908546fb816795931b70395687:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <std::size_t N, class T_IN, class T_OUT>
	:ref:`InputPort<doxid-class_input_port>`<T_IN, T_OUT>* registerInput(const std::string& id)

Creates an :ref:`InputPort <doxid-class_input_port>` and connect it to an input specified by N. Returns the created port.

.. index:: pair: function; getInputByIndex
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1ad7d7562762c78ab77a788a0a92c798a7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`Port<doxid-class_port>`* getInputByIndex(size_t idx)

Returns an :ref:`InputPort <doxid-class_input_port>` by index.

.. index:: pair: function; getInputById
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1aaa0d4f4cc20813efc0887429b2363adb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`Port<doxid-class_port>`* getInputById(const std::string& id)

Returns an :ref:`InputPort <doxid-class_input_port>` by id.

.. index:: pair: function; registerOutput
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a2ad5dd10c60a903a56986bcc52dca8ed:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <std::size_t N, class T>
	:ref:`OutputPort<doxid-class_output_port>`<T>* registerOutput(const std::string& id)

Creates an :ref:`OutputPort <doxid-class_output_port>` and connect it to an output specified by N. Returns the created port.

.. index:: pair: function; getOutputByIndex
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1a3ed48bb6e11cf25d3ccd2b3e5df8084a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <std::size_t N>
	:ref:`Port<doxid-class_port>`* getOutputByIndex()

Returns an :ref:`OutputPort <doxid-class_output_port>` by index.

.. index:: pair: function; getOutputById
.. _doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7_1aaa78f26b09a1bd9c0877d35976a8bf2a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <std::size_t N = 0>
	:ref:`Port<doxid-class_port>`* getOutputById(const std::string& id)

Returns an :ref:`OutputPort <doxid-class_output_port>` by id.

