Release installation
====================

Open a new terminal or update current shell with:

.. code-block:: bash
 $ source ~/.bashrc

* Download repositories (from project or source release):

  * From source release:
  
  	.. code-block:: bash
  	
         $ cd $AEROSTACK2_PATH
         $ vcs import --recursive < ./installers/as2_beta_beetroot.repos
         
  * From project (e.g.):

  	.. code-block:: bash
  	
         $ as2 install mbzirc project
         $ as2 project -n mbzirc

* Install dependencies:

.. code-block:: bash
 $ cd ${HOME}/aerostack2_ws
 $ rosdep update
 $ rosdep install --from-paths src --ignore-src -r -y

* Build:

.. code-block:: bash
 $ as2 build

More information on how to run each project can be found in the README.md located on each project folder.