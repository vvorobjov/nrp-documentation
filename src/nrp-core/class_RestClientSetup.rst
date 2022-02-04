.. index:: pair: class; RestClientSetup
.. _doxid-class_rest_client_setup:

class RestClientSetup
=====================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Singleton. Class to setup RestClient and initialize features such as timeouts, authentications, ... :ref:`More...<details-class_rest_client_setup>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <restclient_setup.h>
	
	class RestClientSetup {
	public:
		// construction
	
		:target:`RestClientSetup<doxid-class_rest_client_setup_1a78112803473a0805700d6c799bb33470>`(const RestClientSetup&);
		:target:`RestClientSetup<doxid-class_rest_client_setup_1aec2bf918e352fc3061d67a2dac21aa59>`(RestClientSetup&&);

		// methods
	
		RestClientSetup& :target:`operator =<doxid-class_rest_client_setup_1aad12d2889ba747023372c086fdfc331d>` (const RestClientSetup&);
		RestClientSetup& :target:`operator =<doxid-class_rest_client_setup_1a91051683495ecec82e5b647e10152d19>` (RestClientSetup&&);
		static RestClientSetup* :ref:`getInstance<doxid-class_rest_client_setup_1a0f0adcbb50b57c33d33f24678855ccbf>`();
		static RestClientSetup* :ref:`resetInstance<doxid-class_rest_client_setup_1a13812a6d75dad3e3f6b0dd2cae698a4e>`();
		static RestClientSetup* :ref:`ensureInstance<doxid-class_rest_client_setup_1a96401412f5adae77c25910b73539fc8c>`();
	};
.. _details-class_rest_client_setup:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Singleton. Class to setup RestClient and initialize features such as timeouts, authentications, ...

Methods
-------

.. index:: pair: function; getInstance
.. _doxid-class_rest_client_setup_1a0f0adcbb50b57c33d33f24678855ccbf:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static RestClientSetup* getInstance()

Get :ref:`RestClientSetup <doxid-class_rest_client_setup>` instance. Returns nullptr if not yet initialized.

.. index:: pair: function; resetInstance
.. _doxid-class_rest_client_setup_1a13812a6d75dad3e3f6b0dd2cae698a4e:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static RestClientSetup* resetInstance()

Resets :ref:`RestClientSetup <doxid-class_rest_client_setup>`.

.. index:: pair: function; ensureInstance
.. _doxid-class_rest_client_setup_1a96401412f5adae77c25910b73539fc8c:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	static RestClientSetup* ensureInstance()

Ensure that :ref:`RestClientSetup <doxid-class_rest_client_setup>` has been initialized.

