.. _examples_dji:

========
DJI OSKD
========

For these examples we are going to use :ref:`DJI Platform <aerial_platform_dji_matrice>`. 
You can install it following the instructions in :ref:`installation section <aerial_platform_dji_matrice_installation>`.

.. figure:: resources/DJI_M300.jpg
   :scale: 25
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

   project_dji_osdk/index.rst

