.. _undefined.clientLoggerService:

===============================
Module: ``clientLoggerService``
===============================


.. contents:: Local Navigation
   :local:

Children
========

.. toctree::
   :maxdepth: 1
   
   clientLoggerModule.module_clientLoggerService-ClientLoggerService
   clientLoggerModule.module_clientLoggerService-ClientLoggerService
   
Description
===========

Service that manages subscriptions to the client logger ros-topic


.. _clientLoggerModule.module_clientLoggerService.logMessage:


Function: ``logMessage``
========================

Public method to log a message with a specifig log type

.. js:function:: logMessage()

    
    :param logMessage(): is a string that includes the message
    :param logMessage(): is a LOG_TYPE that represents the location where the message is displayed
    
.. _clientLoggerModule.module_clientLoggerService.subscribeRosTopic:


Function: ``subscribeRosTopic``
===============================

Private method subscribing to the log ros-topic

.. js:function:: subscribeRosTopic()

    
    :param subscribeRosTopic(): The callback to be called when joint topic messages are received
    
.. _clientLoggerModule.module_clientLoggerService.unsubscribeRosTopic:


Function: ``unsubscribeRosTopic``
=================================

Private method unsubscribing to the log ros-topic

.. js:function:: unsubscribeRosTopic()

    
    
.. _clientLoggerModule.module_clientLoggerService.resetLoggedMessages:


Function: ``resetLoggedMessages``
=================================

Public method that has to be called if log history is collected

.. js:function:: resetLoggedMessages()

    
    
.. _clientLoggerModule.module_clientLoggerService.getLogHistory:


Function: ``getLogHistory``
===========================

Public getter that returns all previously received log messages

.. js:function:: getLogHistory()

    
    

.. _clientLoggerModule.module_clientLoggerService.level:

Member: ``level``: 

.. _clientLoggerModule.module_clientLoggerService.message:

Member: ``message``: 

.. _clientLoggerModule.module_clientLoggerService.duration:

Member: ``duration``: 

.. _clientLoggerModule.module_clientLoggerService.INFO:

Member: ``INFO``: 

.. _clientLoggerModule.module_clientLoggerService.ADVERTS:

Member: ``ADVERTS``: 




