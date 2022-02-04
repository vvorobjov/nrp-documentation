.. index:: pair: class; PipeCommunication
.. _doxid-class_pipe_communication:

class PipeCommunication
=======================

.. toctree::
	:hidden:

Overview
~~~~~~~~

Creates a pipe, used for inter-process communication. Currently used just in Tests. :ref:`More...<details-class_pipe_communication>`


.. ref-code-block:: cpp
	:class: doxyrest-overview-code-block

	#include <pipe_communication.h>
	
	class PipeCommunication {
	public:
		// methods
	
		ssize_t :ref:`readP<doxid-class_pipe_communication_1aa2fcc03de1d67a14cfc5889b5dd863f3>`(
			void* buf,
			size_t count,
			u_int16_t tries = 1,
			u_int16_t sleepSecs = 0
		);
	
		ssize_t :ref:`writeP<doxid-class_pipe_communication_1a93c771f5af561fa8dcfceb387f033a53>`(
			const void* buf,
			size_t count,
			u_int16_t tries = 1,
			u_int16_t sleepSecs = 0
		);
	
		void :ref:`closeRead<doxid-class_pipe_communication_1a6648b42e27f9ac10a84583271aa9ee2b>`();
		void :ref:`closeWrite<doxid-class_pipe_communication_1a39ef151abd08af515a99a7696e3cdcd0>`();
		int :ref:`readFd<doxid-class_pipe_communication_1a519c71b779ddbeb527a6edbf48b63683>`() const;
		int :ref:`writeFd<doxid-class_pipe_communication_1a73b4aac8219049b52fe27305b583d024>`() const;
	};
.. _details-class_pipe_communication:

Detailed Documentation
~~~~~~~~~~~~~~~~~~~~~~

Creates a pipe, used for inter-process communication. Currently used just in Tests.

Methods
-------

.. index:: pair: function; readP
.. _doxid-class_pipe_communication_1aa2fcc03de1d67a14cfc5889b5dd863f3:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ssize_t readP(
		void* buf,
		size_t count,
		u_int16_t tries = 1,
		u_int16_t sleepSecs = 0
	)

Read from pipe.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- buf

		- buffer to read to

	*
		- count

		- Number of bytes to read

	*
		- tries

		- How often to try to read from buffer

	*
		- sleepSecs

		- How long to wait between retries



.. rubric:: Returns:

-1 on error, else number of read bytes

.. index:: pair: function; writeP
.. _doxid-class_pipe_communication_1a93c771f5af561fa8dcfceb387f033a53:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	ssize_t writeP(
		const void* buf,
		size_t count,
		u_int16_t tries = 1,
		u_int16_t sleepSecs = 0
	)

Write to pipe.



.. rubric:: Parameters:

.. list-table::
	:widths: 20 80

	*
		- buf

		- buffer to write from

	*
		- count

		- Number of bytes to write

	*
		- tries

		- How often to try to write from buffer

	*
		- sleepSecs

		- How long to wait between retries



.. rubric:: Returns:

-1 on error, else number of written bytes

.. index:: pair: function; closeRead
.. _doxid-class_pipe_communication_1a6648b42e27f9ac10a84583271aa9ee2b:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void closeRead()

Close Read direction.

.. index:: pair: function; closeWrite
.. _doxid-class_pipe_communication_1a39ef151abd08af515a99a7696e3cdcd0:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	void closeWrite()

Close Write direction.

.. index:: pair: function; readFd
.. _doxid-class_pipe_communication_1a519c71b779ddbeb527a6edbf48b63683:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	int readFd() const

Get Read File Descriptor.

.. index:: pair: function; writeFd
.. _doxid-class_pipe_communication_1a73b4aac8219049b52fe27305b583d024:

.. ref-code-block:: cpp
	:class: doxyrest-title-code-block

	int writeFd() const

Get Write File Descriptor.

