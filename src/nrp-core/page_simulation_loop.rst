.. index:: pair: page; Simulation Loop
.. _doxid-simulation_loop:

Simulation Loop
===============

The simulation loop is the main loop where the synchronization of engines, data transfer and data processing happens. The class responsible for managing its execution is: :ref:`FTILoop <doxid-class_f_t_i_loop>`, which stands for *Fixed Time Increment Loop* and implements the synchronization algorithm summarized below in this page.

On initialization, it creates a :ref:`TransceiverFunctionManager <doxid-class_transceiver_function_manager>` to manage all user-generated TransceiverFunctions. Additionally, it runs the initialization routines of all engines.

During the simulation, a sequence of steps is managed by the :ref:`FTILoop <doxid-class_f_t_i_loop>`. Each step represents a series of data exchange and processing that happen at a given simulation time, *t*. This simulation time is calculated by the :ref:`FTILoop <doxid-class_f_t_i_loop>` on the basis of all individual engine time steps (see below). In order to guarantee an ordered synchronization of the various engines with each other, we thus strongly suggest that the users impose some constraints to the values taken by the engine time steps declared in the experiment configuration (see section :ref:`Simulation Configuration <doxid-simulation_configuration>` hereafter). One such constraint can be that all individual engine time steps be a multiple of the smallest engine time step *dt*. In that case, *t* is always a multiple of the smallest engine time step *dt*. Another such constraint, even stronger, is that every individual time step be the product of the smallest engine time step *dt* by a power of two (i.e. 2 n *dt*). In this case, all engines will synchronize with each other at times that are multiples of the largest individual engine time step.

The way the :ref:`FTILoop <doxid-class_f_t_i_loop>` works is described below:

#. The time step of each engine is checked, and if *t* is a multiple of that engine time step, the engine is added to a list *L*.

#. The loop waits for every engine inside *L* to complete. The simulation as a whole is blocked until they all return.

#. All datapacks required by all active preprocessing and transceiver functions are identified. Those linked to engines in *L* are requested and stored in a local datapack cache. If the datapack already exists in the cache, it is overwritten.

#. Every preprocessing function (PF) linked to every engine inside *L* is executed. DataPacks required for executing these functions are retrieved from the local cache and passed to them. Each PF returns an array of datapacks which is stored in this same cache.

#. TFs linked to engines in *L* are identified and executed. DataPacks required for executing these functions are retrieved from the cache and pass to them. The TFs return arrays with datapacks that should be sent to engines.

#. DataPacks received from TFs are sent to all engines in *L*.

#. A timeout is checked. Should it have occurred, the simulation loop is stopped. Otherwise, *t* is updated to *t+dt* and the cycle restarts.

On shutdown, each engine is issued a shutdown command to close gracefully.

The entire loop is executed within one function, :ref:`FTILoop::runLoop <doxid-class_f_t_i_loop_1adedc0d66c8e286e31c3b00dcd9ee46dc>`. Should any of the above steps fail, an exception is thrown.

