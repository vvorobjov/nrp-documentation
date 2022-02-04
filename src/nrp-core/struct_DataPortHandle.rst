.. index:: pair: struct; DataPortHandle
.. _doxid-struct_data_port_handle:

template struct DataPortHandle
==============================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Helper structure managing data and ports associated with a port id. :ref:`More...<details-struct_data_port_handle>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <input_node.h>
	
	template <class DATA>
	struct DataPortHandle {
		// fields
	
		std::shared_ptr<:ref:`OutputPort<doxid-class_output_port>`<DATA>> :ref:`singlePort<doxid-struct_data_port_handle_1a60baa9e1e75dfaf0f8cec9ec6a5a827c>`;
		std::shared_ptr<:ref:`OutputPort<doxid-class_output_port>`<std::vector<const DATA*>>> :ref:`listPort<doxid-struct_data_port_handle_1a1bd349bb41e0f92cdca7e655b3120ccb>`;

		// construction
	
		:target:`DataPortHandle<doxid-struct_data_port_handle_1a21515822a56c7535f34f78901a768e31>`();
	
		:ref:`DataPortHandle<doxid-struct_data_port_handle_1a88f4b97f4be6a4295284b709567b5cf1>`(
			const std::string& id,
			:ref:`ComputationalNode<doxid-class_computational_node>`* parent,
			size_t queue_size
		);

		// methods
	
		void :ref:`publishLast<doxid-struct_data_port_handle_1a6ce33305d98e7116e5cc700a1dab0a59>`();
		void :ref:`publishAll<doxid-struct_data_port_handle_1a0b861be35f30c81169ed56707968c152>`();
		void :ref:`publishNullandClear<doxid-struct_data_port_handle_1af30f9a932fc24ed94c52b23e07a4f7f3>`();
		bool :ref:`addMsg<doxid-struct_data_port_handle_1ae3a671b0aa9dc323e401a575f3dc1ded>`(const DATA* msg);
		void :ref:`clear<doxid-struct_data_port_handle_1aa032b5c68a23efb842e9eb7c23e2a4cb>`();
		size_t :ref:`size<doxid-struct_data_port_handle_1aae5501de0957feb28653feb44579e83d>`();
	};
.. _details-struct_data_port_handle:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Helper structure managing data and ports associated with a port id.

Fields
------

.. index:: pair: variable; singlePort
.. _doxid-struct_data_port_handle_1a60baa9e1e75dfaf0f8cec9ec6a5a827c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::shared_ptr<:ref:`OutputPort<doxid-class_output_port>`<DATA>> singlePort

:ref:`Port <doxid-class_port>` used to send a single msg.

.. index:: pair: variable; listPort
.. _doxid-struct_data_port_handle_1a1bd349bb41e0f92cdca7e655b3120ccb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	std::shared_ptr<:ref:`OutputPort<doxid-class_output_port>`<std::vector<const DATA*>>> listPort

:ref:`Port <doxid-class_port>` used to send a list of msgs.

Construction
------------

.. index:: pair: function; DataPortHandle
.. _doxid-struct_data_port_handle_1a88f4b97f4be6a4295284b709567b5cf1:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	DataPortHandle(
		const std::string& id,
		:ref:`ComputationalNode<doxid-class_computational_node>`* parent,
		size_t queue_size
	)

Constructor.

Methods
-------

.. index:: pair: function; publishLast
.. _doxid-struct_data_port_handle_1a6ce33305d98e7116e5cc700a1dab0a59:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void publishLast()

Publish last item in data.

.. index:: pair: function; publishAll
.. _doxid-struct_data_port_handle_1a0b861be35f30c81169ed56707968c152:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void publishAll()

Publish all items in data.

.. index:: pair: function; publishNullandClear
.. _doxid-struct_data_port_handle_1af30f9a932fc24ed94c52b23e07a4f7f3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void publishNullandClear()

Publish a null pointer.

.. index:: pair: function; addMsg
.. _doxid-struct_data_port_handle_1ae3a671b0aa9dc323e401a575f3dc1ded:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	bool addMsg(const DATA* msg)

Add a new message to the stored data.

.. index:: pair: function; clear
.. _doxid-struct_data_port_handle_1aa032b5c68a23efb842e9eb7c23e2a4cb:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void clear()

Clear data.

.. index:: pair: function; size
.. _doxid-struct_data_port_handle_1aae5501de0957feb28653feb44579e83d:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	size_t size()

Return the size of stored data.

