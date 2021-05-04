==================================
Class: ``ConstraintBasedLayouter``
==================================


.. contents:: Local Navigation
   :local:

Children
========

.. toctree::
   :maxdepth: 1
   
   
Description
===========




.. _ConstraintBasedLayouter.getPossibleInsertLocations:


Function: ``getPossibleInsertLocations``
========================================

Generates the list of all the possible locations where a new components can be placed

.. js:function:: getPossibleInsertLocations(container)

    
    :param GL_Component container: A golden layout container
    :return GL_Component: The list of possible insert locations
    
.. _ConstraintBasedLayouter.placeComponent:


Function: ``placeComponent``
============================

Places a new component at the required location, orientation
and with the specified percentage in the conxcerned orientation

.. js:function:: placeComponent(component, location, horizontal, percentage)

    
    :param GL_Component_config component: Component config for the new component
    :param GL_Component location: The component
    :param boolean horizontal: Whether the orientation is horizontal or not (not=>vertical)
    :param float percentage: Percentage of space in the concerned orienattion to give to the new component
    
.. _ConstraintBasedLayouter.getSolutionFitness:


Function: ``getSolutionFitness``
================================

Calculates the fitness of placing a component at a giveb location in a certain orientation
Fitness is the delta between the space that can be allocated in the given location as opposed
to the prefered component size as specified in the component config (Smaller fitness is better)

.. js:function:: getSolutionFitness(location, component, horizontal)

    
    :param GL_Component location: The location where to place the new component
    :param GL_Component_config component: The config for the new component to place
    :param boolean horizontal: Whether it is a horizontal placement (or vertical)
    
.. _ConstraintBasedLayouter._getChildSizeSize:


Function: ``_getChildSizeSize``
===============================

Retrieves a component effective sizes and prefered sizes

.. js:function:: _getChildSizeSize(component)

    
    :param * component: Retrieves a component effective sizes and prefered sizes
    :return _getChildSizeSize(component): The component effective sizes and prefered sizes
    
.. _ConstraintBasedLayouter.reequilibrate:


Function: ``reequilibrate``
===========================

Re-equilibrates the space allocated between components in the existing layout,
recusrively and bottom up to approximate prefered sizes as much as possible.

.. js:function:: reequilibrate(container)

    
    :param GL_Component container: The container to re-equilibrate
    :return reequilibrate(container): The container effective sizes and prefered sizes
    
.. _ConstraintBasedLayouter.verifyComponentRequirements:


Function: ``verifyComponentRequirements``
=========================================

Checks if the component config is valid
Throws an exception if it is not the case

.. js:function:: verifyComponentRequirements(component)

    
    :param GL_Component_config component: A Gl component config
    
.. _ConstraintBasedLayouter.addComponent:


Function: ``addComponent``
==========================

Layouter public method to be called by GL service when adding a new component

.. js:function:: addComponent(layout, component)

    
    :param GL_Component layout: Layouter public method to be called by GL service when adding a new component
    :param GL_Component_config component: Layouter public method to be called by GL service when adding a new component
    




