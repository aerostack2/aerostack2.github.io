.. _examples_crazyflie:

==================
Bitcraze Crazyflie
==================

For these examples we are going to use :ref:`Crazyflie Platform <aerial_platform_crazyflie>`. 
You can install it following the instructions in :ref:`installation section <aerial_platform_crazyflie_installation>`.

.. figure:: resources/Crazyflie2_0.jpeg
   :scale: 50
   :class: with-shadow

Every project is installed by the following command:

.. code-block:: bash

    as2 project -n project_name

This will clone the project in the directory ``$AEROSTACK2_PATH/projects/``. 
Reopen the terminal to upload the new project:

.. code-block:: bash

    source ~/.bashrc

Go there by:

.. code-block:: bash

    as2 cd project_name

Here is the list of examples:

.. toctree::
   :maxdepth: 1

   project_crazyflie/index.rst


