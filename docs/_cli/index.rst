CLI
===

Aerostack2 has a CLI to ease the use of the Aerostack2 pipeline. Typing as2 on the command line, you can access the different available tools. Also, it is possible to access the help message with a brief description of the different tools using `as2`, `as2 --help`, or `as2 -h`.

List of commands:

* Build all Aerostack2 (or just the package/packages indicated). It is not necessary to execute this command in the Aerostack2 workspace directory.

.. code-block:: bash

 $ as2 build as2_package

* Change the current directory to the Aerostack2 directory (or the Aerostack2 package directory indicated).

.. code-block:: bash

 $ as2 cd as2_package

* Clean workspace.

.. code-block:: bash

 $ as2 clean as2_package

* List Aerostack2 packages.

.. code-block:: bash

 $ as2 list as2_package

* List and install Aerostack2 projects.

.. code-block:: bash

 $ as2 project as2_project
