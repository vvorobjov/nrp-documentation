========================
Class: ``UpdateChecker``
========================


.. contents:: Local Navigation
   :local:

Children
========

.. toctree::
   :maxdepth: 1
   
   
Description
===========




.. _UpdateChecker.checkForNewVersion:


Function: ``checkForNewVersion``
================================

Public method that compares the current version with the latest version online

Returns: The newer version number if there is a new version, 'null' otherwise

.. js:function:: checkForNewVersion()

    
    
.. _UpdateChecker.getReferenceVersion:


Function: ``getReferenceVersion``
=================================



.. js:function:: getReferenceVersion()

    
    
.. _UpdateChecker.getOnlineRefVersion:


Function: ``getOnlineRefVersion``
=================================



.. js:function:: getOnlineRefVersion()

    
    
.. _UpdateChecker.getLocalVersion:


Function: ``getLocalVersion``
=============================



.. js:function:: getLocalVersion()

    
    
.. _UpdateChecker.getCachedRefVersion:


Function: ``getCachedRefVersion``
=================================

Tries loading the cached information about the NRP latest reference version

Returns: [version, invalid], where 'version' is the cached version and 'invalid' is a boolean validity flag (true => invalid)

.. js:function:: getCachedRefVersion()

    
    
.. _UpdateChecker.saveCachedRefVersion:


Function: ``saveCachedRefVersion``
==================================



.. js:function:: saveCachedRefVersion()

    
    
.. _UpdateChecker.isNewVersion:


Function: ``isNewVersion``
==========================



.. js:function:: isNewVersion()

    
    
.. _UpdateChecker.onExit:


Function: ``onExit``
====================



.. js:function:: onExit()

    
    

.. _UpdateChecker.CHECK_UPDATE_URL:

Member: ``CHECK_UPDATE_URL``: 

.. _UpdateChecker.CHECK_UPDATE_ENABLED:

Member: ``CHECK_UPDATE_ENABLED``: 

.. _UpdateChecker.RELEASE_NOTES_URL:

Member: ``RELEASE_NOTES_URL``: 

.. _UpdateChecker.LOCAL_STORAGE_KEYS:

Member: ``LOCAL_STORAGE_KEYS``: 

.. _UpdateChecker.CHECK_FREQUENCY:

Member: ``CHECK_FREQUENCY``: 

.. _UpdateChecker.nrpFrontendVersion:

Member: ``nrpFrontendVersion``: 

.. _UpdateChecker.$http:

Member: ``$http``: 

.. _UpdateChecker.$q:

Member: ``$q``: 

.. _UpdateChecker.bbpConfig:

Member: ``bbpConfig``: 




