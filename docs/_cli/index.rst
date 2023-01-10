.. _CLI:

CLI
===

Aerostack2 has a CLI to ease the use of the Aerostack2 pipeline. Typing as2 on the command line, you can access the different available tools. Also, it is possible to access the help message with a brief description of the different tools using `as2`, `as2 --help`, or `as2 -h`.

List of commands:

* Build all Aerostack2 (or just the package/packages indicated). It is not necessary to execute this command in the Aerostack2 workspace directory.

  .. code-block:: bash
     :caption: ``as2_package`` and its dependencies are built.
     
     as2 build as2_package
     
  .. code-block:: bash
     :caption: Every package inside Aerostack2 is built.
     
     as2 build

* Change the current directory to the Aerostack2 directory (or the Aerostack2 package directory indicated).

  .. code-block:: bash
     :caption: Takes you to ``as2_package`` directory.
     
     as2 cd as2_package
  
  .. code-block:: bash
     :caption: Takes you to ``$AEROSTACK_PATH`` directory.
     
     as2 cd
  
  .. code-block:: bash
     :caption: Takes you to ``$AEROSTACK_PATH/projects/as2_project`` directory. As long as ``as2_project`` is in the projects folder inside ``$AEROSTACK_PATH``.
     
     as2 cd as2_project

* Clean workspace.

  .. code-block:: bash
     :caption: Removes installed binaries from ``as2_package``.
     
     as2 clean as2_package
  
  .. code-block:: bash
     :caption: Removes all installed binaries from all Aerostack2 packages.
     
     as2 clean -a

* List Aerostack2 packages.

  .. code-block:: bash
     :caption: List all installed packages from Aerostack2.
     
     as2 list

* List and install Aerostack2 projects.

  .. code-block:: bash
     :caption: List available projects.
     
     as2 project list
  
  .. code-block:: bash
     :caption: Install ``as2_project`` in ``$AEROSTACK2_PATH/projects/`` folder.
     
     as2 project as2_project
