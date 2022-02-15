.. _legacy_tensorflow:

TensorFlow and NRP v3.0.5
=========================

=====
Scope
=====

This tutorial will teach you how to install and use TensorFlow within an experiment in the NRP version 3.0.5. Upon completion, you will be able to run a "Hello World" level experiment and be able to explore a more advanced experiment distributed with the NRP.

.. note::
    If you want to use TensorFlow with the latest version of the NRP (v3.2), please refer to :ref:`this tutorial<latest_tensorflow>`.

========================================
Installing TensorFlow for Use in the NRP
========================================

The NRP and TensorFlow have slightly different Python dependency versions for core libraries such as Numpy. Unfortunately, this means that TensorFlow cannot be directly installed in the same virtualenv as the NRP, but it can easily be installed separately and used within the platform.

The most convenient way to install TensorFlow is in an isolated virtualenv. Steps are provided below, but for up-to-date instructions refer to: https://www.tensorflow.org/install/install_linux#installing_with_virtualenv.

1. Ensure you have Python 2.7 pip, dev, and virtualenv libraries installed.

    .. code-block:: bash

        sudo apt-get install python-pip python-dev python-virtualenv

2. Create and activate a virtualenv for TensorFlow, the steps below will assume installation into your ~/.opt directory used by the NRP. If you change this location, you will need to modify later steps.

    .. code-block:: bash

        virtualenv ~/.opt/tensorflow_venv
        source ~/.opt/tensorflow_venv/bin/activate

3. Upgrade pip within your virtualenv, this is required by TensorFlow.

    .. code-block:: bash

        easy_install -U pip

4. Install TensorFlow, select one of the options below depending on your GPU configuration.

    .. code-block:: bash

        pip install --upgrade tensorflow       # select this option if you have no or a non-Nvidia GPU
        pip install --upgrade tensorflow-gpu   # select this option if you have an Nvidia GPU with proper drivers

5. Test your TensorFlow installation.

    .. code-block:: bash

        python - << _EOF
        import tensorflow as tf
        hello = tf.constant('Hello, TensorFlow!')
        sess = tf.Session()
        print(sess.run(hello))
        _EOF

    .. note::

        Installing TensorFlow using the pip library will not include all CPU optimizations that may be possible if compiled natively. If performance is an issue, you may want to explore this option separately.

==========================================
Building a "Hello, TensorFlow!" Experiment
==========================================

Using TensorFlow within an experiment is now fairly straightforward. The structure of the pip installation of TensorFlow is too complex to simply add to your PYTHONPATH as is possible with other libraries. Instead, we will need to use Python's own site-package parsing library.

You can easily embed the above "Hello, TensorFlow!" example within the NRP by adding a new Transfer Function:

    .. code-block:: python

        @nrp.Robot2Neuron()
        def hello_tensorflow(t):

            # make TensorFlow available from home directory installation
            import site, os
            site.addsitedir(os.path.expanduser('~/.opt/tensorflow_venv/lib/python2.7/site-packages'))

            # output "Hello, TensorFlow!" to the graphical logger
            import tensorflow as tf
            hello = tf.constant('Hello, TensorFlow!')
            sess = tf.Session()
            clientLogger.info(sess.run(hello))

This can be added to any of the templated experiments and starting the experiment will produce continuous "Hello, TensorFlow!" messages within the graphical client logger.

    .. image:: hello_tensorflow.png
        :align: center
        :width: 75%

Now you can easily use TensorFlow to perform any task in the NRP v3.0.5!

=============================================================
Further Reading: A More Complex TensorFlow Example Experiment
=============================================================

If you would like to look at a more complex, self-documented example experiment within the NRP - please examine the "
CodeJam 2017 Tutorial - TensorFlow Husky Braitenberg Experiment" experiment.

This is a development maturity level experiment that requires additional TensorFlow model dependencies and editing of experiment files. It uses TensorFlow image classification to semantically interact with the environment and may be useful to examine before building your TensorFlow-based experiment.

Please refer to the README documentation:

    .. code-block:: bash

        $HBP/Experiments/tutorial_tensorflow_husky/README.txt
