.. _getting-started:

.. sectionauthor:: Viktor Vorobev <vorobev@in.tum.de>

Getting started
===============

The :term:`Neurorobotics Platform`, NRP, is a simulation platform that enables you to choose and test different brain models for your robots. You can connect spiking neural networks to simulated robots and run embodiment experiments



NRP usage options
+++++++++++++++++++++++++++

You can either use the online NRP, getting benefits from the high-performance computing and cloud services, or install NRP locally on your machine. More about access options you can read :ref:`here <access-nrp>`. 

Installation
----------------

In order to install the NRP locally, use :ref:`our pre-built Docker images <docker-installation>`. You can also :ref:`build the NRP from the source <source-installation>` (for advanced users and developers).

User interface
++++++++++++++++++++++++++++++

Interaction with simulations is available through NRP UI. This is a browser-based application, that we often call :term:`Frontend`. Use the :ref:`section in the user manual <web-cockpit-manual>` in order to get familiar with the UI.

NRP API
+++++++++++++++++++++++++++++++

For those who needs an API for automation of simulation launching and data acquisition, we have developed so-called :term:`Virtual Coach`, which is implemented in the special Python module :class:`pynrp`. You can install it separately to your machine even if you use the online version of the NRP. Read more about Virtual Coach :ref:`here<user-manual-vc>`.

NPR Tutorials
+++++++++++++++++++

For a quick start with the features of the NRP, use our :ref:`nrp-tutorials` section.

Terminology
++++++++++++++++++++++++++++

First, and *most importantly*, we need to give definitions of some terms.

* An **experiment** is a use case, combining a brain, a robot and an environment. 

* A **simulation** is an instance of an experiment, launched by a particular user, at a certain time, with a predefined timeout.

Thus, you could say that you can *define*, *choose* or *design* an **experiment**, while you *run* a **simulation** of this experiment.

You can always consult our :ref:`nrp-glossary` in order to get familiar with jargon.