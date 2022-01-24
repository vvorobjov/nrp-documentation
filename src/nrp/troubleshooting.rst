.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

.. _nrp-troubleshooting:

NRP Troubleshooting
=================

.. contents:: Table of Contents
    :depth: 2
    :local:

During and after source installation
++++++++++++++++++++++++++++++++++++

Please, refer a :doc:`separate page for troubleshooting source installation problems <dev_troubleshooting>`.


Suspicious console messages
+++++++++++++++++++++++++++


Could not process inbound connection
------------------------------------

.. meta::
    :issue: https://hbpneurorobotics.atlassian.net/browse/NRRPLT-8133
    :issue: https://hbpneurorobotics.atlassian.net/browse/NRRPLT-6683

Problem
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    There are warnings of the following type:

    .. code-block:: bash
        
        [WARN] [1529056864.360605, 594.721000]: Could not process inbound connection: /sm_statemachine_0_1529056842953_frontend_generated is not a publisher of /ros_cle_simulation/cle_error. Topics are [['/ros_cle_simulation/logs', 'cle_ros_msgs/ClientLoggerMessage'], ['/rosout', 'rosgraph_msgs/Log']]{'message_definition': '# This message contains information on a error raised after a user update of some CLE related source code:\n# Transfer function, Smach script, PyNN script\nint32 severity # error severity according to the following constants\nint32 SEVERITY_WARNING=0  # The error might not affect the simulation at all\nint32 SEVERITY_ERROR=1    # The error will lead to a simulation failure if not resolved\nint32 SEVERITY_CRITICAL=2 # The error has lead to simulation failure\n\nstring sourceType # e.g., "Transfer Function"\nstring SOURCE_TYPE_TRANSFER_FUNCTION=Transfer Function\nstring SOURCE_TYPE_STATE_MACHINE=State Machine\nstring errorType # e.g., "NoOrMultipleNames", "Compile", "Loading", "Runtime"\nstring message # description of the error, e.g., "IndentationError: unexpected indent"\n\nstring functionName # python def name of the function causing the error, empty if unavailable\n\n# the following fields are used when a python SyntaxError is raised, strings are left empty otherwise\nint32 lineNumber # line number of the error, -1 if unavailable \nint32 offset # python\'s SyntaxError offset, -1 if unavailable\nstring lineText # text of the line causing the error\nstring fileName # name of the file in which the error is raised (<string> if no such file exists)\n', 'callerid': '/rosbridge_websocket', 'tcp_nodelay': '0', 'md5sum': 'f99ce60bbbecd4c8917fd193ca754a03', 'topic': '/ros_cle_simulation/cle_error', 'type': 'cle_ros_msgs/CLEError'}

Reason
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    It's a rospy warning. It is shown when a Publisher stops before its Subscribers.

    In this case it's by design: The frontend (subscriber) can outlive StateMachines (publisher) since they can be deleted from the SM editor by the user.

Solution
~~~~~~~~~~~~~~~~~~~

    **Ignore**