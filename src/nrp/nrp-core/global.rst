.. _global:
.. index:: pair: namespace; global

Global Namespace
================

.. toctree::
	:hidden:

	namespace_FunctionalNodePolicies.rst
	namespace_InputNodePolicies.rst
	namespace_NGraph.rst
	namespace_OutputNodePolicies.rst
	namespace_boost.rst
	namespace_client.rst
	namespace_gazebo.rst
	namespace_json_converter.rst
	namespace_json_utils.rst
	namespace_nlohmann.rst
	namespace_numpy_json_serializer.rst
	namespace_proto_field_ops.rst
	namespace_python.rst
	namespace_python-2.rst
	namespace_std.rst
	struct_ComputationalGraphHandle.rst
	struct_DataPackIdentifier.rst
	struct_DataPortHandle.rst
	struct_EngineGRPCConfigConst.rst
	struct_EngineJSONConfigConst.rst
	struct_FixedString.rst
	struct_GazeboGrpcConfigConst.rst
	struct_GazeboJSONConfigConst.rst
	struct_NestConfigConst.rst
	struct_NestServerConfigConst.rst
	struct_OpenSimConfigConst.rst
	struct_PyEngineScriptWrapper.rst
	struct_PyRegisterEngineDecorator.rst
	struct_PythonConfigConst.rst
	struct_SimulationParams.rst
	struct_TransceiverDataPackInterfaceWrapper.rst
	struct_TransceiverFunctionData.rst
	struct_TransceiverFunctionSortedResults.rst
	struct_ZipSourceWrapper.rst
	struct_ZipWrapper.rst
	struct_dataConverter.rst
	struct_dataConverter-2.rst
	struct_dataConverter-3.rst
	struct_function_traits.rst
	struct_function_traits-2.rst
	struct_sub_tuple.rst
	struct_sub_tuple-2.rst
	struct_sub_tuple-3.rst
	struct_tuple_array.rst
	struct_tuple_array-2.rst
	class_BasicFork.rst
	class_ComputationalGraph.rst
	class_ComputationalGraphManager.rst
	class_ComputationalNode.rst
	class_CreateDataPackClass.rst
	class_DataPack.rst
	class_DataPackController.rst
	class_DataPackInterface.rst
	class_DataPackProcessor.rst
	class_EmptyLaunchCommand.rst
	class_EngineClient.rst
	class_EngineClientInterface.rst
	class_EngineDataPack.rst
	class_EngineGRPCOptsParser.rst
	class_EngineGrpcClient.rst
	class_EngineGrpcServer.rst
	class_EngineJSONNRPClient.rst
	class_EngineJSONOptsParser.rst
	class_EngineJSONRegistrationServer.rst
	class_EngineJSONServer.rst
	class_EngineLauncherInterface.rst
	class_EngineLauncherManager.rst
	class_EventLoop.rst
	class_F2FEdge.rst
	class_FTILoop.rst
	class_FileFinder.rst
	class_FunctionalNode.rst
	class_FunctionalNode-2.rst
	class_FunctionalNodeFactory.rst
	class_GazeboEngineGrpcNRPClient.rst
	class_GazeboEngineJSONNRPClient.rst
	class_GazeboStepController.rst
	class_InputDummy.rst
	class_InputDummyEdge.rst
	class_InputEngineEdge.rst
	class_InputEngineNode.rst
	class_InputNode.rst
	class_InputPort.rst
	class_InputROSEdge.rst
	class_InputROSNode.rst
	class_JsonDataPackController.rst
	class_LaunchCommand.rst
	class_LaunchCommandInterface.rst
	class_NRPCommunicationController.rst
	class_NRPException.rst
	class_NRPExceptionNonRecoverable.rst
	class_NRPExceptionRecoverable.rst
	class_NRPLogger.rst
	class_NestEngineJSONDataPackController.rst
	class_NestEngineJSONNRPClient.rst
	class_NestEngineServerNRPClient.rst
	class_NestJSONServer.rst
	class_NestKernelDataPackController.rst
	class_NrpCoreServer.rst
	class_OpenSimNRPClient.rst
	class_OutputDummy.rst
	class_OutputDummyEdge.rst
	class_OutputEngineEdge.rst
	class_OutputEngineNode.rst
	class_OutputNode.rst
	class_OutputPort.rst
	class_OutputROSEdge.rst
	class_OutputROSNode.rst
	class_PipeCommunication.rst
	class_PluginManager.rst
	class_Port.rst
	class_PreprocessedDataPack.rst
	class_PreprocessingFunction.rst
	class_ProcessLauncher.rst
	class_ProcessLauncherBasic.rst
	class_ProcessLauncherInterface.rst
	class_ProcessLauncherManager.rst
	class_PtrTemplates.rst
	class_PyEngineScript.rst
	class_PythonEngineJSONDataPackController.rst
	class_PythonEngineJSONNRPClient.rst
	class_PythonEngineJSONNRPClientBase.rst
	class_PythonFunctionalNode.rst
	class_PythonGILLock.rst
	class_PythonInterpreterState.rst
	class_PythonJSONServer.rst
	class_RestClientSetup.rst
	class_SimpleInputEdge.rst
	class_SimpleOutputEdge.rst
	class_SimulationManager.rst
	class_TFManagerHandle.rst
	class_TransceiverDataPackInterface.rst
	class_TransceiverFunction.rst
	class_TransceiverFunctionInterpreter.rst
	class_TransceiverFunctionManager.rst
	class_WCharTConverter.rst
	class_ZipContainer.rst
	class_node_policies_ns.rst

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	
	// namespaces

	namespace :ref:`FunctionalNodePolicies<doxid-namespace_functional_node_policies>`;
	namespace :ref:`InputNodePolicies<doxid-namespace_input_node_policies>`;
	namespace :ref:`NGraph<doxid-namespace_n_graph>`;
	namespace :ref:`OutputNodePolicies<doxid-namespace_output_node_policies>`;
	namespace :ref:`boost<doxid-namespaceboost>`;
	namespace :ref:`client<doxid-namespaceclient>`;
	namespace :ref:`gazebo<doxid-namespacegazebo>`;
	namespace :ref:`json_converter<doxid-namespacejson__converter>`;
	namespace :ref:`json_utils<doxid-namespacejson__utils>`;
	namespace :ref:`nlohmann<doxid-namespacenlohmann>`;
	namespace :ref:`numpy_json_serializer<doxid-namespacenumpy__json__serializer>`;
	namespace :ref:`proto_field_ops<doxid-namespaceproto__field__ops>`;
	namespace :ref:`python<doxid-namespaceboost_1_1python>`;
	namespace :ref:`python<doxid-namespacepython>`;
		namespace :ref:`python::OpensimLib<doxid-namespacepython_1_1_opensim_lib>`;
		namespace :ref:`python::SimManager<doxid-namespacepython_1_1_sim_manager>`;
	namespace :ref:`std<doxid-namespacestd>`;

	// typedefs

	typedef :ref:`DataPackController<doxid-class_data_pack_controller>`<google::protobuf::Message> :target:`ProtoDataPackController<doxid-engine__grpc__server_8h_1a8b6f823dadc78cb7cb8e59f426810363>`;
	typedef :ref:`DataPack<doxid-class_data_pack>`<:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`> :target:`JsonDataPack<doxid-json__datapack_8h_1a38c2ec0652fdeb672e81cd62a3f5aca8>`;
	typedef nlohmann::json :target:`json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`;
	typedef GazeboEngineGrpcNRPClient::EngineLauncher<:ref:`GazeboGrpcConfigConst::EngineType<doxid-struct_gazebo_grpc_config_const_1af5a59d7148915d27cc1e60819bcd637a>`> :target:`GazeboEngineGrpcLauncher<doxid-gazebo__engine__grpc__nrp__client_8h_1af6064c498c8de39b95872d844818d832>`;
	typedef GazeboEngineJSONNRPClient::EngineLauncher<:ref:`GazeboJSONConfigConst::EngineType<doxid-struct_gazebo_j_s_o_n_config_const_1a8712a9c3c63ec96a486092b89e2a6a9a>`> :target:`GazeboEngineJSONLauncher<doxid-gazebo__engine__json__nrp__client_8h_1ade2f849701f29330c8721bdd7dd9239a>`;
	typedef :ref:`DataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`DataPackInterfaceSharedPtr<doxid-datapack__interface_8h_1a9d281e3f02f8d9f2ee2535fe0c0905ca>`;
	typedef :ref:`DataPackInterface::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`DataPackInterfaceConstSharedPtr<doxid-datapack__interface_8h_1a8685cae43af20a5eded9c1e6991451c9>`;
	typedef :ref:`EngineClientInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>`;
	typedef :ref:`EngineClientInterface::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`EngineClientInterfaceConstSharedPtr<doxid-engine__client__interface_8h_1ab36a13459f7053cf753c310ef0b68723>`;
	typedef :ref:`EngineLauncherInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`EngineLauncherInterfaceSharedPtr<doxid-engine__client__interface_8h_1ae3f4035d1af7dc2ad1f8e415922cecbc>`;
	typedef :ref:`EngineLauncherInterface::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`EngineLauncherInterfaceConstSharedPtr<doxid-engine__client__interface_8h_1a63b8cf0500d82e90565c405a55d922d2>`;
	typedef :ref:`EngineLauncherManager::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`EngineLauncherManagerSharedPtr<doxid-engine__launcher__manager_8h_1a9a6ad01577394aa98e8c86ef03269955>`;
	typedef :ref:`EngineLauncherManager::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`EngineLauncherManagerConstSharedPtr<doxid-engine__launcher__manager_8h_1af0948ce319f25e479423423640c94c0c>`;
	typedef :ref:`NRP_ENGINE_LAUNCH_FCN_T<doxid-plugin_8h_1adfa9559edd7fd1dc7a0919efe8057d4c>` :target:`engine_launch_fcn_t<doxid-plugin__manager_8cpp_1a1726ca2cb382de946b359f2619e6e4ac>`;
	typedef :ref:`ProcessLauncherManager<doxid-class_process_launcher_manager>`<:ref:`ProcessLauncherBasic<doxid-class_process_launcher_basic>`> :ref:`MainProcessLauncherManager<doxid-process__launcher__manager_8h_1ae6cafafb9f741f3540b40ee357399f6b>`;
	typedef :ref:`MainProcessLauncherManager::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`MainProcessLauncherManagerSharedPtr<doxid-process__launcher__manager_8h_1ac8db601a55ae90e781d2c9e47a175c98>`;
	typedef :ref:`MainProcessLauncherManager::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`MainProcessLauncherManagerConstSharedPtr<doxid-process__launcher__manager_8h_1afa159d2ccf7ef268b592ebdde7afc49d>`;
	typedef :ref:`EngineClientInterface::datapack_identifiers_set_t<doxid-class_engine_client_interface_1a1700e4b2a4d1334187aa5242a04fd9cd>` :target:`DataPackIdentifiers<doxid-nrp__general__library_2nrp__general__library_2python_2python__module_8cpp_1a52441c5828129e81a5130b8e240cf324>`;
	typedef std::shared_ptr<:ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`> :target:`TransceiverFunctionInterpreterSharedPtr<doxid-transceiver__datapack__interface_8h_1a9b0fefd4c9b42bc5a8d8573051997d41>`;
	typedef std::shared_ptr<const :ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`> :target:`TransceiverFunctionInterpreterConstSharedPtr<doxid-transceiver__datapack__interface_8h_1a5e326233605c29c77d50c0dcc129397e>`;
	typedef :ref:`TransceiverDataPackInterface::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`TransceiverDataPackInterfaceSharedPtr<doxid-transceiver__datapack__interface_8h_1a32d7478b1d3bfae5d84644961d494d1a>`;
	typedef :ref:`TransceiverDataPackInterface::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`TransceiverDataPackInterfaceConstSharedPtr<doxid-transceiver__datapack__interface_8h_1a7da6b0593fd142163e6ec9e002596714>`;
	typedef std::shared_ptr<:ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`> :target:`TransceiverFunctionInterpreterSharedPtr<doxid-transceiver__function__interpreter_8h_1a9b0fefd4c9b42bc5a8d8573051997d41>`;
	typedef std::shared_ptr<const :ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`> :target:`TransceiverFunctionInterpreterConstSharedPtr<doxid-transceiver__function__interpreter_8h_1a5e326233605c29c77d50c0dcc129397e>`;
	typedef std::shared_ptr<:ref:`TransceiverFunctionManager<doxid-class_transceiver_function_manager>`> :target:`TransceiverFunctionManagerSharedPtr<doxid-transceiver__function__manager_8h_1a04a557686c5b3fe7ecd8450b33507ce6>`;
	typedef std::shared_ptr<const :ref:`TransceiverFunctionManager<doxid-class_transceiver_function_manager>`> :target:`TransceiverFunctionManagerConstSharedPtr<doxid-transceiver__function__manager_8h_1a51d373111cf73c3488e4e7b6403ff4b4>`;
	typedef std::shared_ptr<:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`> :target:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>`;
	typedef std::shared_ptr<const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`> :target:`jsonConstSharedPtr<doxid-json__schema__utils_8h_1aefb483b68c65a60571e42f7b3111ddfd>`;
	typedef std::chrono::nanoseconds :target:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>`;
	typedef NestEngineJSONNRPClient::EngineLauncher<:ref:`NestConfigConst::EngineType<doxid-struct_nest_config_const_1afb211b7006d5dbd2758c7da59672f320>`> :target:`NestEngineJSONLauncher<doxid-nest__engine__json__nrp__client_8h_1a1e92567c6f5835ddde06a5ea87973ea4>`;
	typedef NestEngineServerNRPClient::EngineLauncher<:ref:`NestServerConfigConst::EngineType<doxid-struct_nest_server_config_const_1a1323c570ee4727eba77f2eab63fbc48e>`> :target:`NestEngineServerNRPClientLauncher<doxid-nest__engine__server__nrp__client_8h_1ae79121bf5ee683da7907393a8f4e12fd>`;
	typedef OpenSimNRPClient::EngineLauncher<:ref:`OpenSimConfigConst::EngineType<doxid-struct_open_sim_config_const_1afc3f141fb388c88d5aaa161fcbfe8ab8>`> :target:`OpenSimJSONLauncher<doxid-opensim__nrp__client_8h_1ab1ec98f150a3b15a4c2d085f3d7b0af3>`;
	typedef PythonEngineJSONNRPClient::EngineLauncher<:ref:`PythonConfigConst::EngineType<doxid-struct_python_config_const_1a48f0b700a0509b2c5f196e4e13acd263>`> :target:`PythonEngineJSONLauncher<doxid-python__engine__json__nrp__client_8h_1a7ad16934aa49da4bfcc8bda7067caa10>`;
	typedef :ref:`PyEngineScript::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`PyEngineScriptSharedPtr<doxid-py__engine__script_8h_1aa1012624d74f571b3c571e6ff078d68e>`;
	typedef :ref:`PyEngineScript::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`PyEngineScriptConstSharedPtr<doxid-py__engine__script_8h_1a80f0070a03b8e33bf26c5359b7ca419c>`;
	typedef :ref:`FTILoop::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`FTILoopSharedPtr<doxid-fti__loop_8h_1a100a0a52390f14edbda4d9fc0ecec24d>`;
	typedef :ref:`FTILoop::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`FTILoopConstSharedPtr<doxid-fti__loop_8h_1aee94d67e4efd86acf417f306ebf1e766>`;
	typedef :ref:`SimulationManager::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>` :target:`SimulationManagerSharedPtr<doxid-simulation__manager_8h_1a3a52f02f8a0eff7b88ba892ae40a0880>`;
	typedef :ref:`SimulationManager::const_shared_ptr<doxid-class_ptr_templates_1ac36bfa374f3b63c85ba97d8cf953ce3b>` :target:`SimulationManagerConstSharedPtr<doxid-simulation__manager_8h_1a8d63883afd499335b077d05350a2bc8b>`;

	// structs

	struct :ref:`ComputationalGraphHandle<doxid-struct_computational_graph_handle>`;
	struct :ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`;

	template <class DATA>
	struct :ref:`DataPortHandle<doxid-struct_data_port_handle>`;

	struct :ref:`EngineGRPCConfigConst<doxid-struct_engine_g_r_p_c_config_const>`;
	struct :ref:`EngineJSONConfigConst<doxid-struct_engine_j_s_o_n_config_const>`;

	template <std::size_t N>
	struct :ref:`FixedString<doxid-struct_fixed_string>`;

	struct :ref:`GazeboGrpcConfigConst<doxid-struct_gazebo_grpc_config_const>`;
	struct :ref:`GazeboJSONConfigConst<doxid-struct_gazebo_j_s_o_n_config_const>`;
	struct :ref:`NestConfigConst<doxid-struct_nest_config_const>`;
	struct :ref:`NestServerConfigConst<doxid-struct_nest_server_config_const>`;
	struct :ref:`OpenSimConfigConst<doxid-struct_open_sim_config_const>`;
	struct :ref:`PyEngineScriptWrapper<doxid-struct_py_engine_script_wrapper>`;
	struct :ref:`PyRegisterEngineDecorator<doxid-struct_py_register_engine_decorator>`;
	struct :ref:`PythonConfigConst<doxid-struct_python_config_const>`;
	struct :ref:`SimulationParams<doxid-struct_simulation_params>`;
	struct :ref:`TransceiverDataPackInterfaceWrapper<doxid-struct_transceiver_data_pack_interface_wrapper>`;
	struct :ref:`TransceiverFunctionData<doxid-struct_transceiver_function_data>`;
	struct :ref:`TransceiverFunctionSortedResults<doxid-struct_transceiver_function_sorted_results>`;
	struct :ref:`ZipSourceWrapper<doxid-struct_zip_source_wrapper>`;
	struct :ref:`ZipWrapper<doxid-struct_zip_wrapper>`;

	template <class T_OUT>
	struct :ref:`dataConverter<bpy::object, T_OUT><doxid-structdata_converter_3_01bpy_1_1object_00_01_t___o_u_t_01_4>`;

	template <class T_IN>
	struct :ref:`dataConverter<T_IN, bpy::object><doxid-structdata_converter_3_01_t___i_n_00_01bpy_1_1object_01_4>`;

	template <class T_IN, class T_OUT>
	struct :ref:`dataConverter<doxid-structdata_converter>`;

	template <typename T>
	struct :ref:`function_traits<doxid-structfunction__traits>`;

	template <typename R, typename ... Args>
	struct :ref:`function_traits<std::function<R(Args...)>><doxid-structfunction__traits_3_01std_1_1function_3_01_r_07_args_8_8_8_08_4_01_4>`;

	template <std::size_t IDX, typename... Tpack, typename Tuple>
	struct :ref:`sub_tuple<IDX, std::tuple<Tpack...>, Tuple, IDX><doxid-structsub__tuple_3_01_i_d_x_00_01std_1_1tuple_3_01_tpack_8_8_8_01_4_00_01_tuple_00_01_i_d_x_01_4>`;

	template <size_t IDX1, typename... Tpack, typename Tuple, size_t IDX2>
	struct :ref:`sub_tuple<IDX1, std::tuple<Tpack...>, Tuple, IDX2><doxid-structsub__tuple_3_01_i_d_x1_00_01std_1_1tuple_3_01_tpack_8_8_8_01_4_00_01_tuple_00_01_i_d_x2_01_4>`;

	template <size_t, typename, typename, size_t>
	struct :ref:`sub_tuple<doxid-structsub__tuple>`;

	template <typename T, size_t N>
	struct :ref:`tuple_array<doxid-structtuple__array>`;

	template <typename T>
	struct :ref:`tuple_array<T, 0><doxid-structtuple__array_3_01_t_00_010_01_4>`;

	// classes

	class :ref:`BasicFork<doxid-class_basic_fork>`;
	class :ref:`ComputationalGraph<doxid-class_computational_graph>`;
	class :ref:`ComputationalGraphManager<doxid-class_computational_graph_manager>`;
	class :ref:`ComputationalNode<doxid-class_computational_node>`;
	class :ref:`CreateDataPackClass<doxid-class_create_data_pack_class>`;

	template <class DATA_TYPE>
	class :ref:`DataPack<doxid-class_data_pack>`;

	template <class DATA_TYPE>
	class :ref:`DataPackController<doxid-class_data_pack_controller>`;

	class :ref:`DataPackInterface<doxid-class_data_pack_interface>`;
	class :ref:`DataPackProcessor<doxid-class_data_pack_processor>`;
	class :ref:`EmptyLaunchCommand<doxid-class_empty_launch_command>`;

	template <class ENGINE, const char* SCHEMA>
	class :ref:`EngineClient<doxid-class_engine_client>`;

	class :ref:`EngineClientInterface<doxid-class_engine_client_interface>`;
	class :ref:`EngineDataPack<doxid-class_engine_data_pack>`;
	class :ref:`EngineGRPCOptsParser<doxid-class_engine_g_r_p_c_opts_parser>`;

	template <class ENGINE, const char* SCHEMA, class ... MSG_TYPES>
	class :ref:`EngineGrpcClient<doxid-class_engine_grpc_client>`;

	template <class ... MSG_TYPES>
	class :ref:`EngineGrpcServer<doxid-class_engine_grpc_server>`;

	template <class ENGINE, const char* SCHEMA>
	class :ref:`EngineJSONNRPClient<doxid-class_engine_j_s_o_n_n_r_p_client>`;

	class :ref:`EngineJSONOptsParser<doxid-class_engine_j_s_o_n_opts_parser>`;
	class :ref:`EngineJSONRegistrationServer<doxid-class_engine_j_s_o_n_registration_server>`;
	class :ref:`EngineJSONServer<doxid-class_engine_j_s_o_n_server>`;
	class :ref:`EngineLauncherInterface<doxid-class_engine_launcher_interface>`;
	class :ref:`EngineLauncherManager<doxid-class_engine_launcher_manager>`;
	class :ref:`EventLoop<doxid-class_event_loop>`;
	class :ref:`F2FEdge<doxid-class_f2_f_edge>`;
	class :ref:`FTILoop<doxid-class_f_t_i_loop>`;
	class :ref:`FileFinder<doxid-class_file_finder>`;

	template <typename, typename>
	class :ref:`FunctionalNode<doxid-class_functional_node>`;

	template <typename... INPUT_TYPES, typename... OUTPUT_TYPES>
	class :ref:`FunctionalNode<std::tuple<INPUT_TYPES...>, std::tuple<OUTPUT_TYPES...>><doxid-class_functional_node_3_01std_1_1tuple_3_01_i_n_p_u_t___t_y_p_e_s_8_8_8_01_4_00_01std_1_1tuple_3d00278c889f81afbd250c42d83dfd8e7>`;

	class :ref:`FunctionalNodeFactory<doxid-class_functional_node_factory>`;
	class :ref:`GazeboEngineGrpcNRPClient<doxid-class_gazebo_engine_grpc_n_r_p_client>`;
	class :ref:`GazeboEngineJSONNRPClient<doxid-class_gazebo_engine_j_s_o_n_n_r_p_client>`;
	class :ref:`GazeboStepController<doxid-class_gazebo_step_controller>`;
	class :ref:`InputDummy<doxid-class_input_dummy>`;
	class :ref:`InputDummyEdge<doxid-class_input_dummy_edge>`;
	class :ref:`InputEngineEdge<doxid-class_input_engine_edge>`;
	class :ref:`InputEngineNode<doxid-class_input_engine_node>`;

	template <class DATA>
	class :ref:`InputNode<doxid-class_input_node>`;

	template <class T_IN, class T_OUT>
	class :ref:`InputPort<doxid-class_input_port>`;

	template <class MSG_TYPE>
	class :ref:`InputROSEdge<doxid-class_input_r_o_s_edge>`;

	template <class MSG_TYPE>
	class :ref:`InputROSNode<doxid-class_input_r_o_s_node>`;

	class :ref:`JsonDataPackController<doxid-class_json_data_pack_controller>`;

	template <const char* LAUNCH_COMMAND>
	class :ref:`LaunchCommand<doxid-class_launch_command>`;

	class :ref:`LaunchCommandInterface<doxid-class_launch_command_interface>`;
	class :ref:`NRPCommunicationController<doxid-class_n_r_p_communication_controller>`;
	class :ref:`NRPException<doxid-class_n_r_p_exception>`;
	class :ref:`NRPExceptionNonRecoverable<doxid-class_n_r_p_exception_non_recoverable>`;
	class :ref:`NRPExceptionRecoverable<doxid-class_n_r_p_exception_recoverable>`;
	class :ref:`NRPLogger<doxid-class_n_r_p_logger>`;
	class :ref:`NestEngineJSONDataPackController<doxid-class_nest_engine_j_s_o_n_data_pack_controller>`;
	class :ref:`NestEngineJSONNRPClient<doxid-class_nest_engine_j_s_o_n_n_r_p_client>`;
	class :ref:`NestEngineServerNRPClient<doxid-class_nest_engine_server_n_r_p_client>`;
	class :ref:`NestJSONServer<doxid-class_nest_j_s_o_n_server>`;
	class :ref:`NestKernelDataPackController<doxid-class_nest_kernel_data_pack_controller>`;
	class :ref:`NrpCoreServer<doxid-class_nrp_core_server>`;
	class :ref:`OpenSimNRPClient<doxid-class_open_sim_n_r_p_client>`;
	class :ref:`OutputDummy<doxid-class_output_dummy>`;
	class :ref:`OutputDummyEdge<doxid-class_output_dummy_edge>`;
	class :ref:`OutputEngineEdge<doxid-class_output_engine_edge>`;
	class :ref:`OutputEngineNode<doxid-class_output_engine_node>`;

	template <class DATA>
	class :ref:`OutputNode<doxid-class_output_node>`;

	template <class T>
	class :ref:`OutputPort<doxid-class_output_port>`;

	template <class MSG_TYPE>
	class :ref:`OutputROSEdge<doxid-class_output_r_o_s_edge>`;

	template <class MSG_TYPE>
	class :ref:`OutputROSNode<doxid-class_output_r_o_s_node>`;

	class :ref:`PipeCommunication<doxid-class_pipe_communication>`;
	class :ref:`PluginManager<doxid-class_plugin_manager>`;
	class :ref:`Port<doxid-class_port>`;
	class :ref:`PreprocessedDataPack<doxid-class_preprocessed_data_pack>`;
	class :ref:`PreprocessingFunction<doxid-class_preprocessing_function>`;

	template <
		class PROCESS_LAUNCHER,
		const char* LAUNCHER_TYPE,
		class ... LAUNCHER_COMMANDS
	>
	class :ref:`ProcessLauncher<doxid-class_process_launcher>`;

	class :ref:`ProcessLauncherBasic<doxid-class_process_launcher_basic>`;
	class :ref:`ProcessLauncherInterface<doxid-class_process_launcher_interface>`;

	template <class ... PROCESS_LAUNCHERS>
	class :ref:`ProcessLauncherManager<doxid-class_process_launcher_manager>`;

	template <class T>
	class :ref:`PtrTemplates<doxid-class_ptr_templates>`;

	class :ref:`PyEngineScript<doxid-class_py_engine_script>`;
	class :ref:`PythonEngineJSONDataPackController<doxid-class_python_engine_j_s_o_n_data_pack_controller>`;
	class :ref:`PythonEngineJSONNRPClient<doxid-class_python_engine_j_s_o_n_n_r_p_client>`;

	template <class ENGINE, const char* SCHEMA>
	class :ref:`PythonEngineJSONNRPClientBase<doxid-class_python_engine_j_s_o_n_n_r_p_client_base>`;

	class :ref:`PythonFunctionalNode<doxid-class_python_functional_node>`;
	class :ref:`PythonGILLock<doxid-class_python_g_i_l_lock>`;
	class :ref:`PythonInterpreterState<doxid-class_python_interpreter_state>`;
	class :ref:`PythonJSONServer<doxid-class_python_j_s_o_n_server>`;
	class :ref:`RestClientSetup<doxid-class_rest_client_setup>`;

	template <class T_IN, INPUT_C<T_IN> INPUT_CLASS>
	class :ref:`SimpleInputEdge<doxid-class_simple_input_edge>`;

	template <class T_OUT, OUTPUT_C<T_OUT> OUTPUT_CLASS>
	class :ref:`SimpleOutputEdge<doxid-class_simple_output_edge>`;

	class :ref:`SimulationManager<doxid-class_simulation_manager>`;
	class :ref:`TFManagerHandle<doxid-class_t_f_manager_handle>`;
	class :ref:`TransceiverDataPackInterface<doxid-class_transceiver_data_pack_interface>`;
	class :ref:`TransceiverFunction<doxid-class_transceiver_function>`;
	class :ref:`TransceiverFunctionInterpreter<doxid-class_transceiver_function_interpreter>`;
	class :ref:`TransceiverFunctionManager<doxid-class_transceiver_function_manager>`;
	class :ref:`WCharTConverter<doxid-class_w_char_t_converter>`;
	class :ref:`ZipContainer<doxid-class_zip_container>`;
	class :ref:`node_policies_ns<doxid-classnode__policies__ns>`;

	// global variables

	concept :target:`INPUT_C<doxid-input__edge_8h_1a358f550a3cc649a043e912d9f2804a69>` = std::is_base_of_v<:ref:`InputNode<doxid-class_input_node>`<T_IN>, T>;
	concept :target:`OUTPUT_C<doxid-output__edge_8h_1a04afa16ef5b0ed8247b3d3a7d0164651>` = std::is_base_of_v<:ref:`OutputNode<doxid-class_output_node>`<T_OUT>, T>;
	const char :target:`LAUNCH_COMMAND<doxid-basic__fork_8h_1aa53475dce0eeddd9471c0bf490572cdf>`[] = "BasicFork";
	const char :ref:`EmptyLaunchC<doxid-empty__launch__command_8h_1a277c85656c662e54e4b6d3615df535a5>`[] = "EmptyLaunchCommand";
	const char :ref:`Basic<doxid-process__launcher__basic_8h_1aa573f195aab0b34d13cde4599bbe0d57>`[] = "Basic";
	static :ref:`CreateDataPackClass<doxid-class_create_data_pack_class>`* :target:`pCreateDataPack<doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a1c028f01af32b2cf25017cbb3956b6c2>` = nullptr;
	static :ref:`CreateDataPackClass<doxid-class_create_data_pack_class>`* :target:`pCreateDataPack<doxid-nrp__nest__server__engine_2nrp__nest__server__engine_2python_2nrp__nest__python__module_8cpp_1a1c028f01af32b2cf25017cbb3956b6c2>` = nullptr;

	// global functions

	template <class T>
	bool :target:`operator ==<doxid-set__ops_8hpp_1a82b46bf5aac6530c52994d11699b13a6>` (
		const std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	std::set<T> :target:`operator *<doxid-set__ops_8hpp_1a622ece470fbceabfb9974ea5c7d8edbc>` (
		const std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	std::set<T>& :target:`operator +=<doxid-set__ops_8hpp_1a7ec61c46c8e5cb59adb7fed733e1c3a8>` (
		std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	std::set<T>& :target:`operator -=<doxid-set__ops_8hpp_1af2accb7c4f5465e9c27a8d15ae43ba07>` (
		std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	std::set<T> :ref:`operator +<doxid-set__ops_8hpp_1abdba448c0bab02b62cec7d4f25aaf7df>` (
		const std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	std::set<T> :ref:`operator -<doxid-set__ops_8hpp_1a111c59c42c62d18814378652f441b02b>` (
		const std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	std::set<T> :ref:`symm_diff<doxid-set__ops_8hpp_1a8662740216dc497cf997630ef350649c>`(
		const std::set<T>& A,
		const std::set<T>& B
	);

	template <class T, class constT>
	bool :ref:`includes_elm<doxid-set__ops_8hpp_1ab146b3d0f166f924b1f6387b89997437>`(
		const std::set<T>& A,
		constT& a
	);

	template <class T>
	int :target:`intersection_size<doxid-set__ops_8hpp_1a6ce21294f979be5b409561a7c5e7139d>`(
		const std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	int :target:`big_small_intersection_size<doxid-set__ops_8hpp_1ac1ffb52ab9741e6a3317156732666c7a>`(
		const std::set<T>& A,
		const std::set<T>& B
	);

	template <class T>
	int :target:`union_size<doxid-set__ops_8hpp_1a7b9a7022be4769abb342fabcb2338a47>`(const std::set<T>& A, const std::set<T>& B);

	template <class T>
	int :target:`set_difference_size<doxid-set__ops_8hpp_1adbee5fc31f0e1426cb70959ad316603f>`(
		const std::set<T>& A,
		const std::set<T>& B
	);

	void :target:`createPythonGraphFromConfig<doxid-graph__utils_8h_1a0420056ae63d5ba7968b4ed660b3b764>`(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& config,
		const boost::python::dict& globalDict
	);

	std::pair<std::string, std::string> :target:`extractNodePortFromAddress<doxid-graph__utils_8h_1a83461540072958206cec321f7ce0dd53>`(const std::string& address);
	:target:`CREATE_NRP_ENGINE_LAUNCHER<doxid-gazebo__engine__grpc__nrp__client_8h_1a273dd66d4cead4cbb0506a65bcf2e8a0>`(:ref:`GazeboEngineGrpcLauncher<doxid-gazebo__engine__grpc__nrp__client_8h_1af6064c498c8de39b95872d844818d832>`);
	:target:`CREATE_NRP_ENGINE_LAUNCHER<doxid-gazebo__engine__json__nrp__client_8h_1a8868d789314a69982b84d60964ee23c6>`(:ref:`GazeboEngineJSONLauncher<doxid-gazebo__engine__json__nrp__client_8h_1ade2f849701f29330c8721bdd7dd9239a>`);
	const int& :target:`setCmp<doxid-engine__client__interface_8cpp_1a6ed519fca2ae1ad7057a45f168273177>`(int& ref, int val);

	std::shared_ptr<:ref:`DataPackIdentifier<doxid-struct_data_pack_identifier>`> :target:`genDevID<doxid-nrp__general__library_2nrp__general__library_2python_2python__module_8cpp_1aa35016b64c6e81557546e51dc84abd79>`(
		const std::string& name,
		const std::string& engineName
	);

	std::shared_ptr<:ref:`DataPackInterface<doxid-class_data_pack_interface>`> :target:`genDevInterface<doxid-nrp__general__library_2nrp__general__library_2python_2python__module_8cpp_1a04466c6e013424e96d27cf92cd6cd5d0>`(
		const std::string& name,
		const std::string& engineName
	);

	:target:`BOOST_PYTHON_MODULE<doxid-nrp__general__library_2nrp__general__library_2python_2python__module_8cpp_1ad9c5acca6f80373c6921132395ceb500>`(PYTHON_MODULE_NAME);
	:target:`BOOST_PYTHON_MODULE<doxid-nrp__event__loop_2python__module_2python__module_8cpp_1a1981871bc9a884b0f063888b953ab19c>`(EVENT_LOOP_PYTHON_MODULE_NAME);
	static PyObject* :ref:`setPythonError<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a339702e88564271b88687d73262b044e>`(PyObject* type, const std::string& message);

	static PyObject* :ref:`getItemFromJsonArray<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a1042b2ef8c7cf05005c724e770268504>`(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& jsonParent,
		PyObject* index
	);

	static PyObject* :ref:`getItemFromJsonObject<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a6ab1e7a61b3b18445e9798bd508367de>`(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& jsonParent,
		PyObject* index
	);

	static PyObject* :ref:`nlohmannJsonGetItem<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a1fcad1e928c585b2ae0dd1ed6860d321>`(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& jsonParent,
		PyObject* index
	);

	static void :ref:`nlohmannJsonSetItem<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a0fec4a6eccb3514a3bc3313e80b610cb>`(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* jsonParent,
		PyObject* index,
		PyObject* value
	);

	static PyObject* :ref:`nlohmannJsonDump<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a85299d2ff2f2a3da0a5e5eb244326bd4>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json);
	static PyObject* :ref:`nlohmannJsonSize<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a3814bbd3f65b176460b0fed214ade0c7>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json);
	static PyObject* :ref:`nlohmannJsonType<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a58be983cafd9c11494af8de861d08c51>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json);
	static PyObject* :ref:`nlohmannJsonKeys<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a0919db4c2ba59d9d243a4160b8f8d54d>`(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json);
	static void :target:`nlohmannJsonAppend<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a3e50404d676577b154875673c173e8be>`(:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* jsonParent, PyObject* value);
	:target:`BOOST_PYTHON_MODULE<doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a00cd8a58305ec1572e504034c5c74b5d>`(JSON_PYTHON_MODULE_NAME);

	template <std::size_t N>
	:target:`FixedString<doxid-fixed__string_8h_1ac46512ed62abbf6cc5577562b2e48464>`(const char(&) str[N]);

	template <std::size_t N>
	:target:`FixedString<doxid-fixed__string_8h_1aab993b62c59e0cff196604233fe6e4ba>`(const :ref:`FixedString<doxid-struct_fixed_string>`<N>& str);

	:target:`FixedString<doxid-fixed__string_8h_1a1d4c012778efa6fbea05d2d23cb17e48>`();

	template <class T>
	:target:`FixedString<doxid-fixed__string_8h_1a3874a7862656660040ce4edfbcc4dfc8>`(T str);

	std::string :ref:`handle_pyerror<doxid-python__error__handler_8cpp_1aebb4484a92922eeda9619138c81cb919>`();
	std::string :ref:`handle_pyerror<doxid-python__error__handler_8h_1aebb4484a92922eeda9619138c81cb919>`();
	:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`toSimulationTimeFromSeconds<doxid-time__utils_8cpp_1aa169593657be4951694eef18fd2b2734>`(double time);

	double :ref:`getRoundedRunTimeMs<doxid-time__utils_8cpp_1a9d0c24830e4fec08df3551fe7548a8bc>`(
		const :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runTime,
		const float simulationResolutionMs
	);

	template <class vartype, class ratio>
	static :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`toSimulationTime<doxid-time__utils_8h_1afef6027ed6e7d382275448cf1d0a9293>`(vartype time);

	:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` :ref:`toSimulationTimeFromSeconds<doxid-time__utils_8h_1aa169593657be4951694eef18fd2b2734>`(double time);

	template <class vartype, class ratio>
	static vartype :ref:`fromSimulationTime<doxid-time__utils_8h_1a37992c16209fc710bd50d25e407c38f1>`(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` time);

	double :ref:`getRoundedRunTimeMs<doxid-time__utils_8h_1a9d0c24830e4fec08df3551fe7548a8bc>`(
		const :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runTime,
		const float simulationResolutionMs
	);

	uint16_t :ref:`findUnboundPort<doxid-utils_8h_1a337f000ce03ab76de37985f1948d2da4>`(uint16_t startPort);
	void :ref:`appendPythonPath<doxid-utils_8h_1aaeafb1b211dba67ef710bbf359cf34a6>`(const std::string& path);
	int :target:`main<doxid-nrp__nest__engines_2nrp__nest__json__engine_2nest__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>`(int argc, char* argv[]);
	int :target:`main<doxid-nrp__python__json__engine_2python__server__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>`(int argc, char* argv[]);

	static void :target:`loadPlugins<doxid-nrp__simulation_2nrp__simulation__executable_2main_8cpp_1a6b5786857a4c7468fc88fc5b69be155c>`(
		const char* libName,
		:ref:`PluginManager<doxid-class_plugin_manager>`& pluginManager,
		const :ref:`EngineLauncherManagerSharedPtr<doxid-engine__launcher__manager_8h_1a9a6ad01577394aa98e8c86ef03269955>`& engines
	);

	static void :target:`loadEngines<doxid-nrp__simulation_2nrp__simulation__executable_2main_8cpp_1afaf3ec750d2790ebc047b133a2d7c55c>`(
		:ref:`PluginManager<doxid-class_plugin_manager>`& pluginManager,
		:ref:`EngineLauncherManagerSharedPtr<doxid-engine__launcher__manager_8h_1a9a6ad01577394aa98e8c86ef03269955>`& engines,
		const cxxopts::ParseResult& startParams
	);

	static void :target:`runServerMode<doxid-nrp__simulation_2nrp__simulation__executable_2main_8cpp_1a4d2bc9b3d323f088606b91107795709c>`(
		:ref:`EngineLauncherManagerSharedPtr<doxid-engine__launcher__manager_8h_1a9a6ad01577394aa98e8c86ef03269955>`& engines,
		:ref:`MainProcessLauncherManager::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`& processLaunchers,
		:ref:`SimulationManager<doxid-class_simulation_manager>`& manager,
		const std::string& address
	);

	static void :target:`runStandaloneMode<doxid-nrp__simulation_2nrp__simulation__executable_2main_8cpp_1acced5ff1ccb24d9be881cc8093c1b7bb>`(
		:ref:`EngineLauncherManagerSharedPtr<doxid-engine__launcher__manager_8h_1a9a6ad01577394aa98e8c86ef03269955>`& engines,
		:ref:`MainProcessLauncherManager::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`& processLaunchers,
		:ref:`SimulationManager<doxid-class_simulation_manager>`& manager
	);

	static void :target:`runEventLoopMode<doxid-nrp__simulation_2nrp__simulation__executable_2main_8cpp_1a6bd71672efe0da4e5f7bc28167ce6aac>`(
		:ref:`EngineLauncherManagerSharedPtr<doxid-engine__launcher__manager_8h_1a9a6ad01577394aa98e8c86ef03269955>`& engines,
		:ref:`MainProcessLauncherManager::shared_ptr<doxid-class_ptr_templates_1a71a8266f22feaa7154763ceb94e25457>`& processLaunchers,
		:ref:`SimulationManager<doxid-class_simulation_manager>`& manager,
		std::unique_ptr<:ref:`EventLoop<doxid-class_event_loop>`>& eLoop,
		std::chrono::milliseconds& timeout,
		bool runFTILoop
	);

	int :target:`main<doxid-nrp__simulation_2nrp__simulation__executable_2main_8cpp_1a0ddf1224851353fc92bfbff6f499fa97>`(int argc, char* argv[]);
	:target:`CREATE_NRP_ENGINE_LAUNCHER<doxid-nest__engine__json__nrp__client_8h_1af7c694c928fb20f2fcd2aa34cc0b1091>`(:ref:`NestEngineJSONLauncher<doxid-nest__engine__json__nrp__client_8h_1a1e92567c6f5835ddde06a5ea87973ea4>`);
	python::object :target:`CreateDataPack<doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a845e37097987f38a00e0921eeabb48d4>`(python::tuple args, python::dict kwargs);
	void :target:`RegisterDataPack<doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a29ef04a5068442acee81c1de9c94bdcc>`(python::str devName, python::object nodeCollection);
	python::dict :target:`GetDevMap<doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a108d8011a7aff12351532930a2331e6f>`();
	:target:`BOOST_PYTHON_MODULE<doxid-nrp__nest__json__engine_2nrp__nest__json__engine_2python_2nrp__nest__python__module_8cpp_1a99213caf40dd64bea0c122e5ee7e4d84>`(NRP_NEST_PYTHON_MODULE);
	python::object :target:`CreateDataPack<doxid-nrp__nest__server__engine_2nrp__nest__server__engine_2python_2nrp__nest__python__module_8cpp_1a845e37097987f38a00e0921eeabb48d4>`(python::tuple args, python::dict kwargs);
	void :target:`RegisterDataPack<doxid-nrp__nest__server__engine_2nrp__nest__server__engine_2python_2nrp__nest__python__module_8cpp_1a29ef04a5068442acee81c1de9c94bdcc>`(python::str devName, python::object nodeCollection);
	python::dict :target:`GetDevMap<doxid-nrp__nest__server__engine_2nrp__nest__server__engine_2python_2nrp__nest__python__module_8cpp_1a108d8011a7aff12351532930a2331e6f>`();
	:target:`BOOST_PYTHON_MODULE<doxid-nrp__nest__server__engine_2nrp__nest__server__engine_2python_2nrp__nest__python__module_8cpp_1a99213caf40dd64bea0c122e5ee7e4d84>`(NRP_NEST_PYTHON_MODULE);
	:target:`CREATE_NRP_ENGINE_LAUNCHER<doxid-nest__engine__server__nrp__client_8h_1aff9842b0a5cd95156f66e5664d639612>`(:ref:`NestEngineServerNRPClientLauncher<doxid-nest__engine__server__nrp__client_8h_1ae79121bf5ee683da7907393a8f4e12fd>`);

	Py_ssize_t :target:`ExtractIndices<doxid-repeated__field__proxy_8cpp_1a5d172eb5ab6752c4b6c715f7a537c1ea>`(
		PyObject* indices,
		Py_ssize_t& from,
		Py_ssize_t& to,
		Py_ssize_t& step,
		Py_ssize_t length
	);

	:target:`CREATE_NRP_ENGINE_LAUNCHER<doxid-opensim__nrp__client_8h_1a00a5d498b9047571cb8e704e2e644b39>`(:ref:`OpenSimJSONLauncher<doxid-opensim__nrp__client_8h_1ab1ec98f150a3b15a4c2d085f3d7b0af3>`);
	:target:`CREATE_NRP_ENGINE_LAUNCHER<doxid-python__engine__json__nrp__client_8h_1aafdb58696d9d654e5c9c2d55c915dd34>`(:ref:`PythonEngineJSONLauncher<doxid-python__engine__json__nrp__client_8h_1a7ad16934aa49da4bfcc8bda7067caa10>`);
	void :ref:`PyServerRegistration<doxid-nrp__python__engine__module_8cpp_1a5bfae99b75c7c1fd8ac9e7447f3f62d0>`(python::object script);
	:target:`BOOST_PYTHON_MODULE<doxid-nrp__python__engine__module_8cpp_1a49c58d5af6f984868bf10891b1017797>`(NRP_PYTHON_ENGINE_MODULE);
	static :ref:`DataPackProcessor<doxid-class_data_pack_processor>`* :target:`makeHandleFromConfig<doxid-fti__loop_8cpp_1a22348f8cf0c0bf4d0b1c5804fa2ac75b>`(:ref:`jsonSharedPtr<doxid-json__schema__utils_8h_1a1a31aaa02300075f725729b8d8ea57c5>` config);
	static void :target:`runLoopStepAsyncGet<doxid-fti__loop_8cpp_1ad63d48c0c9f35b41a2c2ed074f02daf8>`(:ref:`EngineClientInterfaceSharedPtr<doxid-engine__client__interface_8h_1ac903cea490b8ce188463593e5b5621b3>` engine);

	// macros

	#define :ref:`CREATE_NRP_ENGINE_LAUNCHER<doxid-plugin_8h_1adcd291c2e449ed4d17ddfb6476b1d246>`(engine_launcher_name)
	#define :target:`CREATE_NRP_ENGINE_LAUNCHER_FCN<doxid-plugin_8h_1a205a8995ba665d70e625c8240ffc7092>`
	#define :target:`CREATE_NRP_ENGINE_LAUNCHER_FCN_STR<doxid-plugin_8h_1adaffd84cabef1c786c276992f56facb9>`
	#define :ref:`NPY_NO_DEPRECATED_API<doxid-json__converter_8cpp_1ab6e6ee86736f9ebb56e74ae21bf3ff8a>`
	#define :target:`NRP_ENGINE_LAUNCH_FCN_T<doxid-plugin_8h_1adfa9559edd7fd1dc7a0919efe8057d4c>`
	#define :ref:`NRP_LOGGER_TRACE<doxid-nrp__logger_8h_1a44d0ffe46e0db421ac193bb0eaa6e0f5>`(...)
	#define :target:`SPDLOG_ACTIVE_LEVEL<doxid-nrp__logger_8h_1ae4fda6f71f35120e2ff48157fca961b5>`

.. _details-global:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Typedefs
--------

.. index:: pair: typedef; MainProcessLauncherManager
.. _doxid-process__launcher__manager_8h_1ae6cafafb9f741f3540b40ee357399f6b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	typedef :ref:`ProcessLauncherManager<doxid-class_process_launcher_manager>`<:ref:`ProcessLauncherBasic<doxid-class_process_launcher_basic>`> MainProcessLauncherManager

Type to manage all available process launchers.

Global Variables
----------------

.. index:: pair: variable; EmptyLaunchC
.. _doxid-empty__launch__command_8h_1a277c85656c662e54e4b6d3615df535a5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const char EmptyLaunchC[] = "EmptyLaunchCommand"

Empty Launch Command. A "dummy" launcher that doesn't launch a process. Useful in the cases when the EngineServer is not intended to be launched by the corresponding :ref:`EngineClient <doxid-class_engine_client>`.

.. index:: pair: variable; Basic
.. _doxid-process__launcher__basic_8h_1aa573f195aab0b34d13cde4599bbe0d57:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	const char Basic[] = "Basic"

Basic Process Launcher, for simple process management.

Global Functions
----------------

.. index:: pair: function; operator+
.. _doxid-set__ops_8hpp_1abdba448c0bab02b62cec7d4f25aaf7df:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class T>
	std::set<T> operator + (
		const std::set<T>& A,
		const std::set<T>& B
	)



.. rubric:: Returns:

a new set, the union of A and B.

.. index:: pair: function; operator-
.. _doxid-set__ops_8hpp_1a111c59c42c62d18814378652f441b02b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class T>
	std::set<T> operator - (
		const std::set<T>& A,
		const std::set<T>& B
	)



.. rubric:: Returns:

the A - B: elements in A but not in B.

.. index:: pair: function; symm_diff
.. _doxid-set__ops_8hpp_1a8662740216dc497cf997630ef350649c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class T>
	std::set<T> symm_diff(
		const std::set<T>& A,
		const std::set<T>& B
	)



.. rubric:: Returns:

a new (possibly empty) set, the symmetric difference of A and B. That is, elements in only one set, but not the other. Mathematically, this is A+B - (A\*B)

.. index:: pair: function; includes_elm
.. _doxid-set__ops_8hpp_1ab146b3d0f166f924b1f6387b89997437:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class T, class constT>
	bool includes_elm(
		const std::set<T>& A,
		constT& a
	)



.. rubric:: Returns:

true, if element a is in set A

.. index:: pair: function; setPythonError
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a339702e88564271b88687d73262b044e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* setPythonError(PyObject* type, const std::string& message)

Generates python error with given type and message.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- type

		- Type of exception

	*
		- message

		- Message of the exception



.. rubric:: Returns:

Py_None object

.. index:: pair: function; getItemFromJsonArray
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a1042b2ef8c7cf05005c724e770268504:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* getItemFromJsonArray(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& jsonParent,
		PyObject* index
	)

Returns element stored in json array under given index.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked

	*
		- index

		- Index to be accessed



.. rubric:: Returns:

Object stored under given index

.. index:: pair: function; getItemFromJsonObject
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a6ab1e7a61b3b18445e9798bd508367de:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* getItemFromJsonObject(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& jsonParent,
		PyObject* index
	)

Returns element stored in json object under given key.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked

	*
		- index

		- Key to be accessed



.. rubric:: Returns:

Object stored under given key

.. index:: pair: function; nlohmannJsonGetItem
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a1fcad1e928c585b2ae0dd1ed6860d321:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* nlohmannJsonGetItem(
		const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& jsonParent,
		PyObject* index
	)

Implements python **getitem** method of nlohmann::json.

The function is part of the python interface for nlohmann::json. It allows to access nlohmann::json objects using 'object[index]' notation. Both json arrays and objects cannot be accessed this way.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked

	*
		- index

		- Index (in case of arrays) or key (in case of objects) to be accessed



.. rubric:: Returns:

Object stored under given key or index

.. index:: pair: function; nlohmannJsonSetItem
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a0fec4a6eccb3514a3bc3313e80b610cb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static void nlohmannJsonSetItem(
		:ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`* jsonParent,
		PyObject* index,
		PyObject* value
	)

Implements python **setitem** method of nlohmann::json.

The function is part of the python interface for nlohmann::json. It allows to modify nlohmann::json objects using 'object[key] = value' notation. Json arrays cannot be modified this way.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked

	*
		- index

		- Key to be modified or created

	*
		- value

		- Value to be inserted under the key

.. index:: pair: function; nlohmannJsonDump
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a85299d2ff2f2a3da0a5e5eb244326bd4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* nlohmannJsonDump(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json)

Implements python **str** method of nlohmann::json.

The function is part of the python interface for nlohmann::json.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked



.. rubric:: Returns:

String representation of the object

.. index:: pair: function; nlohmannJsonSize
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a3814bbd3f65b176460b0fed214ade0c7:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* nlohmannJsonSize(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json)

Implements python **len** method of nlohmann::json.

The function is part of the python interface for nlohmann::json.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked



.. rubric:: Returns:

Length of the object as long int

.. index:: pair: function; nlohmannJsonType
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a58be983cafd9c11494af8de861d08c51:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* nlohmannJsonType(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json)

Returns the type of a json object as a string.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked



.. rubric:: Returns:

json type of the object

.. index:: pair: function; nlohmannJsonKeys
.. _doxid-nrp__engine__protocols_2nrp__json__engine__protocol_2python_2python__module_8cpp_1a0919db4c2ba59d9d243a4160b8f8d54d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static PyObject* nlohmannJsonKeys(const :ref:`nlohmann::json<doxid-engine__json__server_8cpp_1ab701e3ac61a85b337ec5c1abaad6742d>`& json)

Implements python keys method of nlohmann::json.

The function is part of the python interface for nlohmann::json.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- json

		- The object of which the method was invoked



.. rubric:: Returns:

List of keys in case of json object, list of indices as strings in case of json array

.. index:: pair: function; handle_pyerror
.. _doxid-python__error__handler_8cpp_1aebb4484a92922eeda9619138c81cb919:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string handle_pyerror()

Read out a properly formatted Python exception string. Only call if a Python exception was thrown.



.. rubric:: Returns:

Returns human-readable error string

.. index:: pair: function; handle_pyerror
.. _doxid-python__error__handler_8h_1aebb4484a92922eeda9619138c81cb919:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::string handle_pyerror()

Read out a properly formatted Python exception string. Only call if a Python exception was thrown.



.. rubric:: Returns:

Returns human-readable error string

.. index:: pair: function; toSimulationTimeFromSeconds
.. _doxid-time__utils_8cpp_1aa169593657be4951694eef18fd2b2734:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` toSimulationTimeFromSeconds(double time)

Converts floating-point seconds into SimulationTime.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- time

		- The value to be converted



.. rubric:: Returns:

SimulationTime object that corresponds to the argument

.. index:: pair: function; getRoundedRunTimeMs
.. _doxid-time__utils_8cpp_1a9d0c24830e4fec08df3551fe7548a8bc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	double getRoundedRunTimeMs(
		const :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runTime,
		const float simulationResolutionMs
	)

Calculates simulation run time rounded to milliseconds, accounting for given resolution.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- runTime

		- Simulation run time that will be rounded

	*
		- simulationResolutionMs

		- Simulation resolution in milliseconds

	*
		- :ref:`NRPException <doxid-class_n_r_p_exception>`

		- When simulation run time is smaller than simulation resolution



.. rubric:: Returns:

Rounded simulation run time

.. index:: pair: function; toSimulationTime
.. _doxid-time__utils_8h_1afef6027ed6e7d382275448cf1d0a9293:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class vartype, class ratio>
	static :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` toSimulationTime(vartype time)

Converts given value to SimulationTime object.

The function is parametrized with two parameters: vartype and ratio. Vartype is the type of variable that is supposed to be converted to SimulationTime. Usually this will be int or float. Ratio is std::ratio class.

.. index:: pair: function; toSimulationTimeFromSeconds
.. _doxid-time__utils_8h_1aa169593657be4951694eef18fd2b2734:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` toSimulationTimeFromSeconds(double time)

Converts floating-point seconds into SimulationTime.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- time

		- The value to be converted



.. rubric:: Returns:

SimulationTime object that corresponds to the argument

.. index:: pair: function; fromSimulationTime
.. _doxid-time__utils_8h_1a37992c16209fc710bd50d25e407c38f1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class vartype, class ratio>
	static vartype fromSimulationTime(:ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` time)

Converts SimulationTime object to specified type and with given ratio.

.. index:: pair: function; getRoundedRunTimeMs
.. _doxid-time__utils_8h_1a9d0c24830e4fec08df3551fe7548a8bc:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	double getRoundedRunTimeMs(
		const :ref:`SimulationTime<doxid-time__utils_8h_1aadff625b29e39dc2d75a7d476d5040c1>` runTime,
		const float simulationResolutionMs
	)

Calculates simulation run time rounded to milliseconds, accounting for given resolution.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- runTime

		- Simulation run time that will be rounded

	*
		- simulationResolutionMs

		- Simulation resolution in milliseconds

	*
		- :ref:`NRPException <doxid-class_n_r_p_exception>`

		- When simulation run time is smaller than simulation resolution



.. rubric:: Returns:

Rounded simulation run time

.. index:: pair: function; findUnboundPort
.. _doxid-utils_8h_1a337f000ce03ab76de37985f1948d2da4:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	uint16_t findUnboundPort(uint16_t startPort)

Searchs for an unbound port starting from startPort. Returns the first unbound port found as a uint16_t.

.. index:: pair: function; appendPythonPath
.. _doxid-utils_8h_1aaeafb1b211dba67ef710bbf359cf34a6:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void appendPythonPath(const std::string& path)

Appends 'path' to PYTHON_PATH env variable.

.. index:: pair: function; PyServerRegistration
.. _doxid-nrp__python__engine__module_8cpp_1a5bfae99b75c7c1fd8ac9e7447f3f62d0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void PyServerRegistration(python::object script)

Calls :ref:`PythonJSONServer::registerScript() <doxid-class_python_j_s_o_n_server_1abbf64f920e8684fe8547eb929c38999e>` without returning a value.

Macros
------

.. index:: pair: define; CREATE_NRP_ENGINE_LAUNCHER
.. _doxid-plugin_8h_1adcd291c2e449ed4d17ddfb6476b1d246:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	#define CREATE_NRP_ENGINE_LAUNCHER(engine_launcher_name)

Create a new engine launcher. Will be used to load a launcher out of a dynamically loaded library.

.. index:: pair: define; NPY_NO_DEPRECATED_API
.. _doxid-json__converter_8cpp_1ab6e6ee86736f9ebb56e74ae21bf3ff8a:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	#define NPY_NO_DEPRECATED_API

Suppresses numpy warnings about deprecated API.

.. index:: pair: define; NRP_LOGGER_TRACE
.. _doxid-nrp__logger_8h_1a44d0ffe46e0db421ac193bb0eaa6e0f5:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	#define NRP_LOGGER_TRACE(...)

trace log macro. It is voided by defining \PRODUCTION_RELEASE

