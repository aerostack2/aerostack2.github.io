.. _troubleshooting:

Troubleshooting
===============

- `Nvidia NX`_

Nvidia NX
---------

Clock skew
**********
Clock skew while colcon building: ``make: warning:  Clock skew detected.  Your build may be incomplete``.

.. code-block:: bash

    sudo date MMddhhmm(YY)

where:

- **MM** month
- **dd** day
- **hh** hour
- **mm** minutes
- **YY** years