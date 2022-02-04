.. index:: pair: class; NRPROSProxy
.. _doxid-class_n_r_p_r_o_s_proxy:

class NRPROSProxy
=================

.. toctree::
	:hidden:

Overview
~~~~~~~~




.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <nrp_ros_proxy.h>
	
	class NRPROSProxy {
	public:
		// construction
	
		:target:`NRPROSProxy<doxid-class_n_r_p_r_o_s_proxy_1aafca8213a11ef608247af2a6af1ad0ab>`(const NRPROSProxy&);
		:target:`NRPROSProxy<doxid-class_n_r_p_r_o_s_proxy_1af0cc8a7aa2c7029b2a770ca429ffc505>`(NRPROSProxy&&);

		// methods
	
		NRPROSProxy& :target:`operator =<doxid-class_n_r_p_r_o_s_proxy_1a7ef034b2a28d85c12b3bbf512354ba29>` (const NRPROSProxy&);
		NRPROSProxy& :target:`operator =<doxid-class_n_r_p_r_o_s_proxy_1a5f0b5b630c04f2ecb8bd36b57e475a86>` (NRPROSProxy&&);
	
		template <class MSG_TYPE>
		void :ref:`subscribe<doxid-class_n_r_p_r_o_s_proxy_1adea2ebec71e11ad584fdcd8ff6affd41>`(
			const std::string& address,
			const boost::function<void(const boost::shared_ptr<MSG_TYPE const>&)>& callback,
			size_t queueSize = 10
		);
	
		template <class MSG_TYPE>
		void :ref:`publish<doxid-class_n_r_p_r_o_s_proxy_1a41e5ba69a2eb3f6e8736daa2fd4b00f8>`(
			const std::string& address,
			const MSG_TYPE& msg,
			size_t queueSize = 10
		);
	
		static NRPROSProxy& :ref:`getInstance<doxid-class_n_r_p_r_o_s_proxy_1a5f5c1175db12af47131803ef27c91902>`();
		static NRPROSProxy& :ref:`resetInstance<doxid-class_n_r_p_r_o_s_proxy_1a0dcc94aa91ef5949eb097e7abc85ba05>`();
	};
.. _details-class_n_r_p_r_o_s_proxy:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~



Methods
-------

.. index:: pair: function; subscribe
.. _doxid-class_n_r_p_r_o_s_proxy_1adea2ebec71e11ad584fdcd8ff6affd41:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class MSG_TYPE>
	void subscribe(
		const std::string& address,
		const boost::function<void(const boost::shared_ptr<MSG_TYPE const>&)>& callback,
		size_t queueSize = 10
	)

Subscribe to ROS topic 'address' with callback function 'callback'.

.. index:: pair: function; publish
.. _doxid-class_n_r_p_r_o_s_proxy_1a41e5ba69a2eb3f6e8736daa2fd4b00f8:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	template <class MSG_TYPE>
	void publish(
		const std::string& address,
		const MSG_TYPE& msg,
		size_t queueSize = 10
	)

Publishes 'msg' to ROS topic 'address'.

.. index:: pair: function; getInstance
.. _doxid-class_n_r_p_r_o_s_proxy_1a5f5c1175db12af47131803ef27c91902:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static NRPROSProxy& getInstance()

Get singleton instance of :ref:`NRPROSProxy <doxid-class_n_r_p_r_o_s_proxy>`.

.. index:: pair: function; resetInstance
.. _doxid-class_n_r_p_r_o_s_proxy_1a0dcc94aa91ef5949eb097e7abc85ba05:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static NRPROSProxy& resetInstance()

Reset singleton instance.

