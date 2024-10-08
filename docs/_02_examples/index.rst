.. _examples:

========
Examples
========

In this section, severals examples of how to use Aerostack2 are shown. Each example is self-contained in a 
repository, also known as **as2_project**.

.. warning:: Projects main branch is linked with Aerostack2 main branch. If you want to use a specific Aerostack2 binary version, you should checkout the project tag that matches the Aerostack2 version.

.. _examples_prerequisites:

-------------
Prerequisites
-------------

Every project in Aerostack2 uses `tmux <https://github.com/tmux/tmux/wiki>`_ 
and `tmuxinator <https://github.com/tmuxinator/tmuxinator>`_ to get a more organized execution. Install it with:

.. code-block:: bash
    
    sudo apt install tmux tmuxinator iputils-ping -y


.. _examples_basics:

---------------
Basics Examples
---------------

.. toctree::
   :maxdepth: 2

   multirotor_simulator/index.rst
   gazebo/index.rst
   crazyflie/index.rst
   tello/index.rst
   dji/index.rst
   pixhawk/index.rst
