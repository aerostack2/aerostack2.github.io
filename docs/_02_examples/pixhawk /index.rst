.. _examples_pixhawk:

=========
Pixhawk 4
=========

For these examples we are going to use :ref:`Pixhawk Platform <aerial_platform_px4>`. 
You can install it following the instructions in :ref:`installation section <aerial_platform_px4_installation>`.

.. figure:: resources/px4_vision.jpg
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

   project_px4_vision/index.rst
