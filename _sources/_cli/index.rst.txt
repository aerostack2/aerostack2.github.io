.. _CLI:

CLI
===

Aerostack2 has a CLI to ease the use of the Aerostack2 pipeline. Typing as2 on the command line, you can access the different available tools. Also, it is possible to access the help message with a brief description of the different tools using `as2`, `as2 --help`, or `as2 -h`.

List of commands:

* Build all Aerostack2 (or just the package/packages indicated). It is not necessary to execute this command in the Aerostack2 workspace directory.

  .. code-block:: bash

     as2 build as2_package
     
  Expected behavior: ``as2_package`` and its dependencies are built.
  
  .. code-block:: bash

     as2 build
     
  Expected behavior: Every package inside Aerostack2 is built.

* Change the current directory to the Aerostack2 directory (or the Aerostack2 package directory indicated).

  .. code-block:: bash

     as2 cd as2_package
     
  Expected behavior: takes you to ``as2_package`` directory.
  
  .. code-block:: bash

     as2 cd
     
  Expected behavior: takes you to ``$AEROSTACK_PATH`` directory.
  
  .. code-block:: bash

     as2 cd as2_project
     
  Expected behavior: takes you to ``$AEROSTACK_PATH/projects/as2_project`` directory. As long as ``as2_project`` is in the projects folder inside ``$AEROSTACK_PATH``.

* Clean workspace.

  .. code-block:: bash

     as2 clean as2_package
     
  Expected behavior: removes installed binaries from ``as2_package``.
  
  .. code-block:: bash

     as2 clean -a
     
  Expected behavior: removes all installed binaries from all Aerostack2 packages.

* List Aerostack2 packages.

  .. code-block:: bash

     as2 list
   
  Expected behavior: list all installed packages from Aerostack2.

* List and install Aerostack2 projects.

  .. code-block:: bash

     as2 project list
     
  Expected behavior: list available projects.
  
  .. code-block:: bash

     as2 project as2_project
     
  Expected behavior: install ``as2_project`` in ``$AEROSTACK2_PATH/projects/`` folder.
