.. index:: pair: class; NGraph::tGraph
.. _doxid-class_n_graph_1_1t_graph:

template class NGraph::tGraph
=============================

.. toctree::
	:hidden:

	enum_NGraph_tGraph_line_type.rst

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <ngraph.hpp>
	
	template <typename T>
	class tGraph {
	public:
		// typedefs
	
		typedef T :target:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`;
		typedef T :target:`value_type<doxid-class_n_graph_1_1t_graph_1addb8e5aa5779f80e19e368eef9448e8c>`;
		typedef std::pair<:ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`, :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`> :target:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`;
		typedef std::set<:ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`> :target:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`;
		typedef std::set<:ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`> :target:`edge_set<doxid-class_n_graph_1_1t_graph_1a18a0c72c7ef57d877cab05bba5cf4d9b>`;
		typedef std::pair<:ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`, :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`> :target:`in_out_edge_sets<doxid-class_n_graph_1_1t_graph_1a9822c284aa101d90d3bd613c7e05eaa4>`;
		typedef std::map<:ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`, :ref:`in_out_edge_sets<doxid-class_n_graph_1_1t_graph_1a9822c284aa101d90d3bd613c7e05eaa4>`> :target:`adj_graph<doxid-class_n_graph_1_1t_graph_1a0036686b7a2d33d0c2aeca3f6715495d>`;
		typedef edge_set::iterator :target:`edge_iterator<doxid-class_n_graph_1_1t_graph_1a1b710c759e101724f3544b374ebb0179>`;
		typedef edge_set::const_iterator :target:`const_edge_iterator<doxid-class_n_graph_1_1t_graph_1a6e189d2dd98fbe980b830d898eea5247>`;
		typedef vertex_set::iterator :target:`vertex_iterator<doxid-class_n_graph_1_1t_graph_1ad5c343cc3c50b291b35fb48147db3250>`;
		typedef vertex_set::const_iterator :target:`const_vertex_iterator<doxid-class_n_graph_1_1t_graph_1a5ed9ca8ad9676b3c7abd3880f735ce80>`;
		typedef vertex_set::iterator :target:`vertex_neighbor_iterator<doxid-class_n_graph_1_1t_graph_1aeb3fd2cc092aa7ecb4b749ee4e78f9ab>`;
		typedef vertex_set::const_iterator :target:`vertex_neighbor_const_iterator<doxid-class_n_graph_1_1t_graph_1af8b0bc7076b28fb145b267b8faad195e>`;
		typedef adj_graph::iterator :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>`;
		typedef adj_graph::const_iterator :target:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>`;
		typedef :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` :target:`node_iterator<doxid-class_n_graph_1_1t_graph_1af9638aed082c5e33cf4064f158f66d59>`;
		typedef :ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` :target:`const_node_iterator<doxid-class_n_graph_1_1t_graph_1a4a4c169ed8eecce822b2d18b21082ce7>`;

		// enums
	
		enum :ref:`line_type<doxid-class_n_graph_1_1t_graph_1a4a14eff6bcee7d8c0528c13d5e99ad71>`;

		// construction
	
		:target:`tGraph<doxid-class_n_graph_1_1t_graph_1acd9199cd41933c1492669a6b0ad16877>`();
		:target:`tGraph<doxid-class_n_graph_1_1t_graph_1a548e93259bcece749bed2d6375049be9>`(std::istream& s);
		:target:`tGraph<doxid-class_n_graph_1_1t_graph_1a70928f9e49923184ee26a228ebd23ff5>`(const tGraph& B);
		:target:`tGraph<doxid-class_n_graph_1_1t_graph_1a8267a8db43eae2dd34d1d50db57e821c>`(const :ref:`edge_set<doxid-class_n_graph_1_1t_graph_1a18a0c72c7ef57d877cab05bba5cf4d9b>`& E);

		// methods
	
		unsigned int :target:`num_vertices<doxid-class_n_graph_1_1t_graph_1a6aff4cfa2819f372ce4c7030c9709b82>`() const;
		unsigned int :target:`num_nodes<doxid-class_n_graph_1_1t_graph_1a5bbc421c6e9181fbd42ca1255b6fa739>`() const;
		unsigned int :target:`num_edges<doxid-class_n_graph_1_1t_graph_1a25a5ca5600c3c4d4ae58f945981b5561>`() const;
		:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` :target:`begin<doxid-class_n_graph_1_1t_graph_1a1fa503849355f65f3d235b895a3f6086>`();
		:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` :target:`begin<doxid-class_n_graph_1_1t_graph_1a957235ebde0cfda1bdfacdd0db212d73>`() const;
		:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` :target:`end<doxid-class_n_graph_1_1t_graph_1a78775c7c87001e8ceb5668cec6d40dfb>`();
		:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` :target:`end<doxid-class_n_graph_1_1t_graph_1aca77137373df2e5313f434326ffc355a>`() const;
		void :target:`clear<doxid-class_n_graph_1_1t_graph_1add822443b93ee9ef745ee3b8ab12065b>`();
		:ref:`vertex_neighbor_iterator<doxid-class_n_graph_1_1t_graph_1aeb3fd2cc092aa7ecb4b749ee4e78f9ab>` :target:`out_neighbors_begin<doxid-class_n_graph_1_1t_graph_1af4aaf8e70f041bea0759caa9133f5fe6>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a);
		:ref:`vertex_neighbor_const_iterator<doxid-class_n_graph_1_1t_graph_1af8b0bc7076b28fb145b267b8faad195e>` :target:`out_neighbors_begin<doxid-class_n_graph_1_1t_graph_1a58a0b9135d7fdc9bd9bccb37f477ea12>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		:ref:`vertex_neighbor_iterator<doxid-class_n_graph_1_1t_graph_1aeb3fd2cc092aa7ecb4b749ee4e78f9ab>` :target:`out_neighbors_end<doxid-class_n_graph_1_1t_graph_1a85890eeb0085d0079824acc5a8c0240b>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a);
		:ref:`vertex_neighbor_const_iterator<doxid-class_n_graph_1_1t_graph_1af8b0bc7076b28fb145b267b8faad195e>` :target:`out_neighbors_end<doxid-class_n_graph_1_1t_graph_1a90ed49e6b1a6b555713370d8cc3a11c9>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		bool :target:`is_undirected<doxid-class_n_graph_1_1t_graph_1a512ce90c67d43855c0f6b131a90dfbce>`() const;
		bool :target:`is_directed<doxid-class_n_graph_1_1t_graph_1a0533e8f86aceaa221f64d835178fd966>`() const;
		void :target:`set_undirected<doxid-class_n_graph_1_1t_graph_1a374bc267c08ca80f7bece5f2230848d0>`();
		:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` :target:`find<doxid-class_n_graph_1_1t_graph_1a94305809ed5bd6531d73e74d5b63055f>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a);
		:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` :target:`find<doxid-class_n_graph_1_1t_graph_1a15801328dc81c8eff05ce09dd4094b28>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`in_neighbors<doxid-class_n_graph_1_1t_graph_1a58f5d07e8dfe45d237d093c5bc11ad53>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		:ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`in_neighbors<doxid-class_n_graph_1_1t_graph_1a0ca32078d83236c669a6e0beb7d93759>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a);
		const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`out_neighbors<doxid-class_n_graph_1_1t_graph_1acffca17cd25301de3d6aa8a132c4431f>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		:ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`out_neighbors<doxid-class_n_graph_1_1t_graph_1acb50f260b7b391b7c6a73a640bacc08d>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a);
		unsigned int :target:`in_degree<doxid-class_n_graph_1_1t_graph_1a1e6a83e13c11c16599567f6d50e658c2>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		unsigned int :target:`out_degree<doxid-class_n_graph_1_1t_graph_1afa6b411ff31557d044b02f87bbe50999>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		unsigned int :target:`degree<doxid-class_n_graph_1_1t_graph_1a717a15800fec8ebefde8d2d853390a0b>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		bool :target:`isolated<doxid-class_n_graph_1_1t_graph_1a0352a3bfc0fc6d2ce59395d1f0064550>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		void :target:`insert_vertex<doxid-class_n_graph_1_1t_graph_1ae7e728e1249dc0fcb4e2ebec1e4ae6dd>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a);
	
		void :target:`insert_new_vertex_inout_list<doxid-class_n_graph_1_1t_graph_1a5fb33cf26902838865ac86fc51123a96>`(
			const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a,
			const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& IN,
			const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& OUT
		);
	
		void :target:`insert_edge_noloop<doxid-class_n_graph_1_1t_graph_1a3227d649839908a87afbd80316cc493f>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa, :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pb);
		void :target:`insert_edge<doxid-class_n_graph_1_1t_graph_1a5ee540bb69a2bdd199b63af489439545>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa, :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pb);
		void :target:`insert_edge_noloop<doxid-class_n_graph_1_1t_graph_1ad8ef6d934af47685338257274e421bab>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a, const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& b);
		void :target:`insert_edge<doxid-class_n_graph_1_1t_graph_1a7e62b364e0a03fb416facee8b51d99da>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a, const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& b);
		void :target:`insert_undirected_edge<doxid-class_n_graph_1_1t_graph_1a6a0b8b06f321534275d982ae500a4df3>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a, const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& b);
		void :target:`insert_edge<doxid-class_n_graph_1_1t_graph_1a7aa85e0fe919a7c6fafac8051271421b>`(const :ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`& E);
		void :target:`insert_undirected_edge<doxid-class_n_graph_1_1t_graph_1a90d80d85d509b09c932fde138fb0aae0>`(const :ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`& E);
		bool :target:`remove_edge<doxid-class_n_graph_1_1t_graph_1a5af6f7dbadd1f355987bca99d210ebdb>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa, :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pb);
		void :target:`remove_edge<doxid-class_n_graph_1_1t_graph_1ac1fff890e65f3f1654a79a8574b13378>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a, const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& b);
		void :target:`remove_edge<doxid-class_n_graph_1_1t_graph_1ac64968018b02ebd689cb9acb96409327>`(const :ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`& E);
		void :target:`remove_undirected_edge<doxid-class_n_graph_1_1t_graph_1af18958f28aaa6be4aa3ba3e273e5dae3>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a, const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& b);
		void :target:`remove_undirected_edge<doxid-class_n_graph_1_1t_graph_1aa4d5dc7efdb07adb6de626f625701478>`(const :ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`& e);
		void :target:`remove_vertex<doxid-class_n_graph_1_1t_graph_1ae3ddcc68cfd75c8a8c11a47fc1a449b9>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa);
		void :target:`remove_vertex_set<doxid-class_n_graph_1_1t_graph_1adfdabb4a748158e58d1a19ed1ef4fdf1>`(const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& V);
		void :target:`remove_vertex<doxid-class_n_graph_1_1t_graph_1a33aa30bb79f01806408465f1250cac0d>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a);
		bool :ref:`includes_vertex<doxid-class_n_graph_1_1t_graph_1a260849918a2806a4a9aa1cfad0471e41>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const;
		bool :ref:`includes_edge<doxid-class_n_graph_1_1t_graph_1a463b923ee1e3604407837150f17f1a74>`(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a, const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& b) const;
		bool :target:`includes_edge<doxid-class_n_graph_1_1t_graph_1a8d364cdfbd44f0804a91bdcda683b31b>`(const :ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`& e) const;
		std::vector<:ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`> :ref:`edge_list<doxid-class_n_graph_1_1t_graph_1a4f7e150eadbb8845e8f858255022a037>`() const;
		tGraph& :target:`plus_eq<doxid-class_n_graph_1_1t_graph_1af5ad33beb237d62d6a36e1559f50bce1>`(const tGraph& B);
		tGraph :target:`intersect<doxid-class_n_graph_1_1t_graph_1ac2ddbf2173c45a06dbccece30cacb98f>`(const tGraph& B) const;
		tGraph :target:`operator *<doxid-class_n_graph_1_1t_graph_1ad899ac4d991eae39c7cd1d8a7b424ac0>` (const tGraph& B) const;
		tGraph :target:`minus<doxid-class_n_graph_1_1t_graph_1a7b00a9cf016faa75da9dcc429fe503d3>`(const tGraph& B) const;
		tGraph :target:`operator -<doxid-class_n_graph_1_1t_graph_1ae09dcf65e16d50b26515716ec52fb71f>` (const tGraph& B) const;
		tGraph :target:`plus<doxid-class_n_graph_1_1t_graph_1ab9d3103f422170bce7c7a2c952c56024>`(const tGraph& B) const;
		tGraph :target:`operator +<doxid-class_n_graph_1_1t_graph_1aceb173f4fd1cfb252a33d978c62ff49f>` (const tGraph& B) const;
		tGraph& :target:`operator +=<doxid-class_n_graph_1_1t_graph_1a329c3e80609a8bf3c2f6e00ab8298e3d>` (const tGraph& B);
		tGraph :ref:`subgraph<doxid-class_n_graph_1_1t_graph_1ab685e653704db79f06216d0d5977f153>`(const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& A) const;
		unsigned int :target:`subgraph_size<doxid-class_n_graph_1_1t_graph_1ae8f6d218e22f2271aab306e10c252201>`(const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& A) const;
		double :target:`subgraph_sparsity<doxid-class_n_graph_1_1t_graph_1a99c4a072f1445173ca8f36b711f60954>`(const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& A) const;
		void :target:`print<doxid-class_n_graph_1_1t_graph_1aa9a8cb7d229675823a308ab361b849e2>`() const;
		void :ref:`absorb<doxid-class_n_graph_1_1t_graph_1a1844ff9c48370c79147c84384f400c80>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa, :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pb);
		:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` :ref:`smart_absorb<doxid-class_n_graph_1_1t_graph_1aa3752935ebcd6e4688b52cd0f6d1211f>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa, :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pb);
		:ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>` :target:`smart_absorb<doxid-class_n_graph_1_1t_graph_1a35015ce8e51b4301e69d542c21d26c46>`(:ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>` a, :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>` b);
		void :target:`absorb<doxid-class_n_graph_1_1t_graph_1ace64fe566c1df5b77770b4f3ff9ceca0>`(:ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>` a, :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>` b);
		static const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& :ref:`node<doxid-class_n_graph_1_1t_graph_1a826e79cbe8d540de7f044c6ae841dd5f>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& :target:`node<doxid-class_n_graph_1_1t_graph_1a2f6c033bad7dd6e4883de81510901aec>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& :target:`node<doxid-class_n_graph_1_1t_graph_1a4bc654a0d0ddba8df6048e4eac5a1729>`(:ref:`const_vertex_iterator<doxid-class_n_graph_1_1t_graph_1a5ed9ca8ad9676b3c7abd3880f735ce80>` p);
		static const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`in_neighbors<doxid-class_n_graph_1_1t_graph_1a820f6a863d8d838cd4403b9156bc959a>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`in_neighbors<doxid-class_n_graph_1_1t_graph_1aca61e35bcd5af8b66b3c87673771508d>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static :ref:`const_vertex_iterator<doxid-class_n_graph_1_1t_graph_1a5ed9ca8ad9676b3c7abd3880f735ce80>` :target:`in_begin<doxid-class_n_graph_1_1t_graph_1a3152b5d0315b94d8cb1b24a70988d990>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static :ref:`const_vertex_iterator<doxid-class_n_graph_1_1t_graph_1a5ed9ca8ad9676b3c7abd3880f735ce80>` :target:`in_end<doxid-class_n_graph_1_1t_graph_1a89d7cf140e48b1d37b6534035872ab88>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`out_neighbors<doxid-class_n_graph_1_1t_graph_1a1bd0815f5c00d624dbed8bddffab19ae>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static :ref:`const_vertex_iterator<doxid-class_n_graph_1_1t_graph_1a5ed9ca8ad9676b3c7abd3880f735ce80>` :target:`out_begin<doxid-class_n_graph_1_1t_graph_1a43c8b50d4a2a21c5e673510e76fa303e>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static :ref:`const_vertex_iterator<doxid-class_n_graph_1_1t_graph_1a5ed9ca8ad9676b3c7abd3880f735ce80>` :target:`out_end<doxid-class_n_graph_1_1t_graph_1af485a8c7b28e7aa81fbc7cf65316dc21>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& :target:`out_neighbors<doxid-class_n_graph_1_1t_graph_1aab1fb0576d1eb5604779bf0f7ad6d60d>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static :ref:`vertex_iterator<doxid-class_n_graph_1_1t_graph_1ad5c343cc3c50b291b35fb48147db3250>` :target:`out_begin<doxid-class_n_graph_1_1t_graph_1a8c782dadf12b75e66f8db04c60763cbe>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static unsigned int :target:`num_edges<doxid-class_n_graph_1_1t_graph_1a1a850ec5708546dfca02d8a78d41a878>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static unsigned int :target:`num_edges<doxid-class_n_graph_1_1t_graph_1a988ba97d4db2eaa9d9e0ef28658f0d8d>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static unsigned int :ref:`out_degree<doxid-class_n_graph_1_1t_graph_1acd67259b93d9ec2de6ca03bf19e027bd>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static unsigned int :target:`out_degree<doxid-class_n_graph_1_1t_graph_1a855dcfd91171425d09a60593bd222a2b>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static unsigned int :ref:`in_degree<doxid-class_n_graph_1_1t_graph_1a5c269fb5cb63c82496bb9c59d6c289b1>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static unsigned int :target:`in_degree<doxid-class_n_graph_1_1t_graph_1addc7a2aa0c3aa9b2c21cf50b27ec0d63>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static unsigned int :target:`degree<doxid-class_n_graph_1_1t_graph_1a22f2c477b95c7a7bbf4ad117f1905d7c>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static unsigned int :target:`degree<doxid-class_n_graph_1_1t_graph_1a4325f877a4ae56bdfcac3e47d21c5af4>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
		static bool :target:`isolated<doxid-class_n_graph_1_1t_graph_1a5c55e2cba9992aa34e2d2d5d5de43059>`(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p);
		static bool :target:`isolated<doxid-class_n_graph_1_1t_graph_1a2f28e0205a15e21585a4fcd4515fccff>`(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` p);
	
		static std::istream& :target:`read_line<doxid-class_n_graph_1_1t_graph_1ae5f3534e1f30716abf209cede47cff8c>`(
			std::istream& s,
			T& v1,
			T& v2,
			std::string& line,
			:ref:`line_type<doxid-class_n_graph_1_1t_graph_1a4a14eff6bcee7d8c0528c13d5e99ad71>`& t
		);
	};

	// direct descendants

	class :ref:`ComputationalGraph<doxid-class_computational_graph>`;
.. _details-class_n_graph_1_1t_graph:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Typedefs
--------

.. index:: pair: typedef; iterator
.. _doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	typedef adj_graph::iterator iterator

.. code-block:: cpp

	tGraph::iterator refers to pair<const vertex, in_out_edge_sets> .

.. code-block:: none


	:ref:`tGraph::const_iterator <doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p = G.begin();

.. code-block:: none

	:ref:`tGraph::vertex <doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>` a = node(p);
	:ref:`tGraph::vertex_set <doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`  &in_edges = in_neighbors(p);
	:ref:`tGraph::vertex_set <doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`  &out_edges = out_neighbors(p);

.. code-block:: none

Methods
-------

.. index:: pair: function; includes_vertex
.. _doxid-class_n_graph_1_1t_graph_1a260849918a2806a4a9aa1cfad0471e41:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool includes_vertex(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a) const

Is vertex 'a' included in graph?



.. rubric:: Returns:

true, if vertex a is present; false, otherwise.

.. index:: pair: function; includes_edge
.. _doxid-class_n_graph_1_1t_graph_1a463b923ee1e3604407837150f17f1a74:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool includes_edge(const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& a, const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& b) const

Is edge (a,b) included in graph?



.. rubric:: Returns:

true, if edge is present; false, otherwise.

.. index:: pair: function; edge_list
.. _doxid-class_n_graph_1_1t_graph_1a4f7e150eadbb8845e8f858255022a037:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::vector<:ref:`edge<doxid-class_n_graph_1_1t_graph_1a6c85cd9c55a19c3c052f176fe719fdfc>`> edge_list() const

Create a new representation of graph as a list of vertex pairs (a,b).



.. rubric:: Returns:

std::vector<edge> a vector (list) of vertex pairs

.. index:: pair: function; subgraph
.. _doxid-class_n_graph_1_1t_graph_1ab685e653704db79f06216d0d5977f153:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	tGraph subgraph(const :ref:`vertex_set<doxid-class_n_graph_1_1t_graph_1a9e0a5df1ac9a2e6df94431aeaf610b3e>`& A) const



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- A

		- vertex set of nodes (subset of G)



.. rubric:: Returns:

a new subgraph containing all nodes of A

.. index:: pair: function; absorb
.. _doxid-class_n_graph_1_1t_graph_1a1844ff9c48370c79147c84384f400c80:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void absorb(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa, :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pb)

abosrb(a,b): 'a' absorbs 'b', b gets removed from graph

.. index:: pair: function; smart_absorb
.. _doxid-class_n_graph_1_1t_graph_1aa3752935ebcd6e4688b52cd0f6d1211f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` smart_absorb(:ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pa, :ref:`iterator<doxid-class_n_graph_1_1t_graph_1a6e446a33b74e5c0c39fb6c50a4f07cec>` pb)

c smart_abosrb(a,b): 'a' absorbs 'b', or b aborbs a, depending on whichever causes the least amount of graph updates

.. index:: pair: function; node
.. _doxid-class_n_graph_1_1t_graph_1a826e79cbe8d540de7f044c6ae841dd5f:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static const :ref:`vertex<doxid-class_n_graph_1_1t_graph_1a63a04bf8bfc7cf968be524208f49fdee>`& node(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p)



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- p

		- :ref:`tGraph::const_iterator <doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>`

.. index:: pair: function; out_degree
.. _doxid-class_n_graph_1_1t_graph_1acd67259b93d9ec2de6ca03bf19e027bd:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static unsigned int out_degree(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p)



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- p

		- :ref:`tGraph::const_iterator <doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>`



.. rubric:: Returns:

number of edges going out (out-degree) at node pointed to by p.

.. index:: pair: function; in_degree
.. _doxid-class_n_graph_1_1t_graph_1a5c269fb5cb63c82496bb9c59d6c289b1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static unsigned int in_degree(:ref:`const_iterator<doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>` p)



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- p

		- :ref:`tGraph::const_iterator <doxid-class_n_graph_1_1t_graph_1a64864813622245ff9412c784232f2f99>`



.. rubric:: Returns:

number of edges going out (out-degree) at node pointed to by p.

