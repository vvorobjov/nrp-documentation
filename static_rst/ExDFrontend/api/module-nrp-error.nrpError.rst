.. _module-nrp-error.nrpError:

=======================
Namespace: ``nrpError``
=======================

Member Of :doc:`module-nrp-error`

.. contents:: Local Navigation
   :local:

Children
========

.. toctree::
   :maxdepth: 1
   
   
Description
===========

``nrpError`` provides helper functions that all return an
``nrpError`` instance given a context object.


.. _module-nrp-error.nrpError.error:


Function: ``error``
===================

Build a ``nrpError`` instance from the provided options.

- param  {Object} options argument passed to ``nrpError`` constructor
- return {nrpError} the resulting error

.. js:function:: error(options)

    
    :param object options: [description]
    :return object: [description]
    
.. _module-nrp-error.nrpError.httpError:


Function: ``httpError``
=======================

return a `nrpError` instance built from a HTTP response.

In an ideal case, the response contains json data with an error object.
It also fallback to a reason field and fill default error message for
standard HTTP status error.

.. js:function:: httpError(response)

    
    :param HttpResponse response: Angular $http Response object
    :return nrpError: a valid nrpError
    




