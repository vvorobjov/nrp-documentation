============================
Class: ``TipTooltipService``
============================


.. contents:: Local Navigation
   :local:

Children
========

.. toctree::
   :maxdepth: 1
   
   
Description
===========




.. _TipTooltipService.initialize:


Function: ``initialize``
========================



.. js:function:: initialize()

    
    
.. _TipTooltipService.reset:


Function: ``reset``
===================

Resets the service by re-initializing it and resets the tipClusters. The re-initialization is called with the original
constructor parameters. Attention: Mutations on them are not reset.

.. js:function:: reset()

    
    
.. _TipTooltipService.initializeTipClusterIfNotYetExistent:


Function: ``initializeTipClusterIfNotYetExistent``
==================================================

Initializes a tip cluster with the specified identifier if such tip cluster does not already exists.

.. js:function:: initializeTipClusterIfNotYetExistent(identifier)

    
    :param String identifier: Initializes a tip cluster with the specified identifier if such tip cluster does not already exists.
    
.. _TipTooltipService.loadLocalPreferences:


Function: ``loadLocalPreferences``
==================================

Loads the local preferences.

.. js:function:: loadLocalPreferences()

    
    
.. _TipTooltipService.setLocalTipPreferences:


Function: ``setLocalTipPreferences``
====================================

Sets the local preferences for the specified tip. Returns a promise for this task.

.. js:function:: setLocalTipPreferences(tip)

    
    :param * tip: Sets the local preferences for the specified tip. Returns a promise for this task.
    
.. _TipTooltipService.setLocalHiddenPreferences:


Function: ``setLocalHiddenPreferences``
=======================================

Sets the local hidden preferences for the module. Returns a promise for this task.

.. js:function:: setLocalHiddenPreferences()

    
    
.. _TipTooltipService.setHidden:


Function: ``setHidden``
=======================

Sets the hidden property and automatically updates the local storage item.

.. js:function:: setHidden(hide)

    
    :param Boolean hide: Sets the hidden property and automatically updates the local storage item.
    
.. _TipTooltipService.tipToType:


Function: ``tipToType``
=======================

Returns the corresponding type string of the specified tip if it is present in the current tip codes.
Returns an empty string otherwise.

.. js:function:: tipToType(tip)

    
    :param Object tip: Returns the corresponding type string of the specified tip if it is present in the current tip codes.
    Returns an empty string otherwise.
    
.. _TipTooltipService.setCurrentTip:


Function: ``setCurrentTip``
===========================

Set the current tip of the tip cluster with the specified identifier to the specified tip.

.. js:function:: setCurrentTip(tip, tipClusterIdentifier)

    
    :param Object tip: Set the current tip of the tip cluster with the specified identifier to the specified tip.
    :param String tipClusterIdentifier: Set the current tip of the tip cluster with the specified identifier to the specified tip.
    
.. _TipTooltipService.resetAllTipCodeTipPreferences:


Function: ``resetAllTipCodeTipPreferences``
===========================================

Resets the display properties of all tip code tips. Also resets the hidden property unless otherwise specified.

.. js:function:: resetAllTipCodeTipPreferences()

    
    
.. _TipTooltipService.displayTip:


Function: ``displayTip``
========================

Should the specified tip be displayed?

.. js:function:: displayTip()

    
    
.. _TipTooltipService.someTipsAreNotDisplayedInTipCluster:


Function: ``someTipsAreNotDisplayedInTipCluster``
=================================================

Is at least one tip **not** displayed in the tip cluster with the specified identifier?

.. js:function:: someTipsAreNotDisplayedInTipCluster(tipClusterIdentifier)

    
    :param String tipClusterIdentifier: Is at least one tip **not** displayed in the tip cluster with the specified identifier?
    
.. _TipTooltipService.someTipsAreDisplayedInTipCluster:


Function: ``someTipsAreDisplayedInTipCluster``
==============================================

Is at least one tip displayed in the tip cluster with the specified identifier?

.. js:function:: someTipsAreDisplayedInTipCluster(tipClusterIdentifier)

    
    :param String tipClusterIdentifier: Is at least one tip displayed in the tip cluster with the specified identifier?
    

.. _TipTooltipService.ready:

Member: ``ready``: 

.. _TipTooltipService.tipCodesFactory:

Member: ``tipCodesFactory``: 

.. _TipTooltipService.tipClustersFactory:

Member: ``tipClustersFactory``: 

.. _TipTooltipService.DEFAULT_TIP_CLUSTER_IDENTIFIER:

Member: ``DEFAULT_TIP_CLUSTER_IDENTIFIER``: 

.. _TipTooltipService.$sce:

Member: ``$sce``: 

.. _TipTooltipService.hidden:

Member: ``hidden``: 

.. _TipTooltipService.tipCodes:

Member: ``tipCodes``: 

.. _TipTooltipService.tipClusters:

Member: ``tipClusters``: 

.. _TipTooltipService.tipClusters[undefined]:

Member: ``tipClusters[undefined]``: 

.. _TipTooltipService.ready:

Member: ``ready``: 

.. _TipTooltipService.tipClusters[undefined]:

Member: ``tipClusters[undefined]``: 

.. _TipTooltipService.hidden:

Member: ``hidden``: 

.. _TipTooltipService.hidden:

Member: ``hidden``: 




