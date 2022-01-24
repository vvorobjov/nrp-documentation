.. index:: pair: class; python::OpensimLib::OpensimInterface
.. _doxid-classpython_1_1_opensim_lib_1_1_opensim_interface:

class python::OpensimLib::OpensimInterface
==========================================

.. toctree::
	:hidden:




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	class OpensimInterface: public object {
	public:
		// fields
	
		static float :target:`step_size<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1ada4b02a51cb1dd9d8e70b3ce2609545f>` =  0.001;
		static  :target:`model<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1acabe14d6f1be8f3c5a052055e3f5c258>` =  None;
		static  :target:`state<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1abbd62c1535b0eb733eede6873995a29d>` =  None;
		static  :target:`state0<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1ad2d29041edab4a2441f6af1179ea4e0d>` =  None;
		static list :target:`joints<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1abdea88861a82a1386acce73d6fa548d3>` =  [];
		static list :target:`bodies<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a835b31a14d8d601b7282abfee4afe36a>` =  [];
		static  :target:`brain<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a05c1540f4c100602b3af7949598109ad>` =  None;
		static  :target:`manager<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a58db34f4946fa5793edfb5d09d15dce4>` =  None;
		static bool :target:`verbose<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a84c6b5433833dfecb677112d83ce3f61>` =  False;
		static int :target:`n_step<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a49581574c8e8fcf28db3f09135b2d3a3>` =  0;
		static  :target:`state_desc_n_step<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a823ca5a2f06776c8cc8a03c4d87b19f1>` =  None;
		static  :target:`prev_state_desc<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a5d5b2dc0cfd6fcd74983ee7500cc58dc>` =  None;
		static  :target:`state_desc<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1ae9ef7b57d7df41815852db429a42ca20>` =  None;
		static int :target:`integrator_accuracy<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1ad4491f4a225460b7b316b4b910c67981>` =  5e-5;
		static  :target:`jointSet<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a9dcca2cfdc8ad19d97d1997d33f29489>` =  None;
		static  :target:`forceSet<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1aa14ed672b7f389f6d87d005e30868c0d>` =  None;
		static list :target:`maxforces<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a325940100ee8d9838a86f0af62076bed>` =  [];
		static list :target:`curforces<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a7b45f34ae2a75ab1ec6da1ccace04055>` =  [];
		 :target:`step_size<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a21874e5c8b7a28e77c1fdb6fef85bcfe>`;
		 :target:`muscleSet<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a8a339bbd0dc43cf9b3d31c525094d82f>`;
		 :target:`n_step<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a242d085d444e7831f978a438a29cae9e>`;

		// methods
	
		def :target:`__init__<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1af39567b17f125b16a6d8f878a7f7b999>`(
			self self,
			model_name model_name,
			start_visualizer start_visualizer,
			time_step time_step
		);
	
		def :target:`run_one_step<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a6abe4e34c3936bb8366ac1b06955df9c>`(self self, action action);
		def :target:`reset<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1abb6d6e65bcd2155d56c128589bef7000>`(self self);
		def :target:`actuate<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a27a142390d580dbd5d2bd5868f208889>`(self self, action action);
		def :target:`get_model_properties<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a3e4f0799329174b51be08b1670d5d11f>`(self self, p_type p_type);
		def :target:`get_model_property<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a1dcb6f7cf4aa87a36341a44cb1589590>`(self self, p_name p_name, p_type p_type);
		def :target:`get_sim_time<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a8eb37e32b8157453c205933d456bd95d>`(self self);
		def :target:`reset_manager<doxid-classpython_1_1_opensim_lib_1_1_opensim_interface_1a24098cb3655a238ec88381fd42298577>`(self self);
	};
