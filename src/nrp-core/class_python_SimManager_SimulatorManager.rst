.. index:: pair: class; python::SimManager::SimulatorManager
.. _doxid-classpython_1_1_sim_manager_1_1_simulator_manager:

class python::SimManager::SimulatorManager
==========================================

.. toctree::
	:hidden:

.. code-block:: cpp

	This class will receive the information simulator need from NRP engine script,
	and then start and run different simulators with python API


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	class SimulatorManager: public object {
	public:
		// fields
	
		 :target:`time_step<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1ae96214b56479bdb000f574fee6adce44>`;
		 :target:`sim_interface<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1a502555801815cbbe6ff8bba56a05ea43>`;
		 :target:`stepStart<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1a375757539f24a49b6f5c6885e5fccb65>`;

		// methods
	
		def :target:`__init__<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1ab0439f6d7943e5e7372bf3e5b2872b13>`(self self, configureVal configureVal);
		def :target:`reset<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1aea5a7da832b68cefabeaee60f46265f8>`(self self);
		def :target:`run_step<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1a9aca03fab1c9849c3f1024e5ed3af4f4>`(self self, action action);
		def :target:`get_model_properties<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1a7b12094ae28d3a2863707a7e05b9eac1>`(self self, datapack_type datapack_type);
	
		def :target:`get_model_property<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1a792cfa762953dbfdb368e538e3acd25e>`(
			self self,
			datapack_name datapack_name,
			datapack_type datapack_type
		);
	
		def :target:`get_sim_time<doxid-classpython_1_1_sim_manager_1_1_simulator_manager_1ad02e522752235cc9ff8d4fd717a0de26>`(self self);
	};
