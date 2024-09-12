.. _development_guide_new_pkg:

------------------------
Developing a New Package
------------------------


.. _development_guide_new_pkg_architecture:

Architecture of a New Package
=============================

Structure of a C++ Package
--------------------------

Every new C++ package should have the following architecture:

.. code-block::

    /package_name
        /include
            /package_name
                ### header files (.hpp) ###
                package_name.hpp
                header_files.hpp
        /launch
        /src
            ### implementation_files (.cpp)###
            package_name.cpp
            implementation_files.cpp
            ### main_files (_node.cpp) ###
            package_name_node.cpp
        /tests
            CMakeLists.txt
            ### test_files (_test.cpp, _gtest.cpp) ###
            test_executable_test.cpp
            google_test_gtest.cpp
        CmakeLists.txt
        package.xml
        README.md
        LICENSE
        CONTRIBUTING.md
        .gitignore

Note that every executable test file must end with ``_test.cpp`` and every Google test file must end with ``_gtest.cpp``.

Simple packages with just one node should have its only node named as the package. Packages with more than one node or
implementation files may have different naming.

Structure of a Python Package
-----------------------------

Every new Python package should have the following architecture:

.. code-block::

    /package_name
        /package_name
            __init__.py
            ### source_files (.py) ###
            package_name.py
        /launch
        /tests
            ### test_files (test_***.py) ###
            test_file.py
        setup.cfg
        setup.py
        package.xml
        README.md
        LICENSE
        CONTRIBUTING.md
        .gitignore

Structure of a Hybrid Package
-----------------------------

A package that includes both Python and C++ files should have the following architecture:

.. code-block::

    /package_name
        /package_name
            __init__.py
            ### python_source_files(.py) ###
            package_name.py
        /include
            /package_name
                ### C++ header files(.hpp) ###
                package_name.hpp
        
        /launch
        /src
            ### C++ implementation_files(.cpp) ###
            package_name.cpp
            ### C++ main_files (_node.cpp) ###
            package_name_node(.cpp)
        /tests
            ### test_files ###
            test_file.py
        CMakeList.txt
        package.xml
        README.md
        LICENSE
        CONTRIBUTING.md
        .gitignore

.. _development_guide_new_pkg_code_style:

Code Style
==========

Aerostack2 mainly follows the `ROS 2 code style and language versions <https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html>`_. 
There are, though, some exceptions, as ROS 2 claims to follow Google code style but uses snake_case
instead of camelCase. Aerostack2 uses Google code style.


.. _development_guide_new_pkg_code_style_c++:

C++
---

Code should follows the `google c++ style guide <https://google.github.io/styleguide/cppguide.html>`_.

Every file should start with the Licence text, which looks like this:

.. code-block:: c++

    // Copyright 2024 Universidad Politécnica de Madrid
    //
    // Redistribution and use in source and binary forms, with or without
    // modification, are permitted provided that the following conditions are met:
    //
    //    * Redistributions of source code must retain the above copyright
    //      notice, this list of conditions and the following disclaimer.
    //
    //    * Redistributions in binary form must reproduce the above copyright
    //      notice, this list of conditions and the following disclaimer in the
    //      documentation and/or other materials provided with the distribution.
    //
    //    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
    //      contributors may be used to endorse or promote products derived from
    //      this software without specific prior written permission.
    //
    // THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    // AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    // IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    // ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    // LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    // CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    // SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    // INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    // CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    // ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    // POSSIBILITY OF SUCH DAMAGE.

After the License text, files should include a header like this:

.. code-block:: c++

    /**
    * @file file_name.cpp
    *
    * File description
    *
    * @author Author name <author email>
    */

Documentation will be generate using `Doxygen <https://www.doxygen.nl/manual/docblocks.html>`_.
Therefore, header files should include a comment over every definition in order to generate the documentation properly.
The comments made in the current nodes are writing using Javadoc style.

Please, do not use std::cout or similar functions to show messages in command line. Instead, use logging macros like RCLCPP_INFO from ROS logging library.



.. _development_guide_new_pkg_code_style_python:

Python
------

Code should follow the `PEP 8 <https://peps.python.org/pep-0008/>`_ and `PEP 257 <https://peps.python.org/pep-0257/>`_ guidelines.

.. code-block:: c++

    #!/usr/bin/env python3

    # Copyright 2024 Universidad Politécnica de Madrid
    #
    # Redistribution and use in source and binary forms, with or without
    # modification, are permitted provided that the following conditions are met:
    #
    #    * Redistributions of source code must retain the above copyright
    #      notice, this list of conditions and the following disclaimer.
    #
    #    * Redistributions in binary form must reproduce the above copyright
    #      notice, this list of conditions and the following disclaimer in the
    #      documentation and/or other materials provided with the distribution.
    #
    #    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
    #      contributors may be used to endorse or promote products derived from
    #      this software without specific prior written permission.
    #
    # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    # AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
    # IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
    # ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
    # LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
    # CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
    # SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
    # INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
    # CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
    # ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    # POSSIBILITY OF SUCH DAMAGE.

After the License text, files should include a header like this:

.. code-block:: c++

    """Module docstring."""

    __authors__ = 'Author'
    __copyright__ = 'Copyright (c) 2024 Universidad Politécnica de Madrid'
    __license__ = 'BSD-3-Clause'
    __version__ = '0.1.0'

.. _development_guide_new_pkg_test:

Test
====

To be used in Aerostack2, every package must pass, at least, the Code Style tests. Additional functional tests might be added to a package to provide a validation of its functionalities.


.. _development_guide_new_pkg_test_style:

Code Style Test
---------------

Aerostack2 uses `ament_lint <https://github.com/ament/ament_lint>`_ to perform style checks over the packages files.

For configuring style tests, this must be added on the ``CMakeList.txt`` of your package:

.. code-block:: cmake

    if(BUILD_TESTING)
        find_package(ament_lint_auto REQUIRED)
        ament_lint_auto_find_test_dependencies()
    endif()

Also, these packages must be added to the ``package.xml``

.. code-block::

        <test_depend>ament_lint_auto</test_depend>
        <test_depend>ament_lint_common</test_depend>

The tests that are performed as part of ament_lint_common can be found `here <https://github.com/ament/ament_lint/blob/humble/ament_lint_common/doc/index.rst>`_.

Some test dependencies are also required and can be installed by running:

.. code-block:: bash

    apt-get install python3-rosdep python3-pip python3-colcon-common-extensions python3-colcon-mixin ros-dev-tools -y
    apt-get install python3-flake8-builtins python3-flake8-comprehensions python3-flake8-docstrings python3-flake8-import-order python3-flake8-quotes -y

The package can now be compiled running:

.. code-block:: bash

    as2 build <package_name>

And to run the tests, execute:

.. code-block:: bash

    as2 test -v <package_name>

The ``-v`` flag will print all the details of the test run, including information about the tests that did not pass and the specific erros that occurred.

.. _development_guide_new_pkg_test_style_CLI:

Running Individual Tests on CLI
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Ament_lint includes a series of CLI commands with which the tests can be run separately. This might be helpful for fixing the package and eliminate the errors.
These CLI tools can be found on the folder of the specific test on the `ament_lint repository <https://github.com/ament/ament_lint>`_.

· Running and Passing 'ament_uncrustify'
""""""""""""""""""""""""""""""""""""""""""

The test ``ament_uncrustify`` can be launched by running:

.. code-block:: bash

    ament_uncrustify --reformat <path/to/package/directory>/*

Or simply by running the next command from the directory of your package:

.. code-block:: bash

    ament_uncrustify --reformat

Either way, the ``--reformat`` flag will automatically reformat the files in which erros have been found, apart from notifying which are these files.

· Running and Passing 'ament_copyright'
""""""""""""""""""""""""""""""""""""""""""

The test ``ament_copyright`` can be launched by running the next command from your package directory:

.. code-block:: bash

    ament_copyright --add-missing "Universidad Politécnica de Madrid" bsd_3clause

The flag ``--add-missing`` will add the Licence text to all the files that do not include one.

· Running and Passing 'ament_pep257'
""""""""""""""""""""""""""""""""""""""

The test ``ament_pep257`` can be launched by running the next command from your package directory:

.. code-block:: bash

    ament_pep257

The third-party autoformatter `docformatter <https://github.com/PyCQA/docformatter>`_ can be used to help passing this test. It can be installed executing

.. code-block:: bash

    pip install --upgrade docformatter

and launched over the ``.py`` files by running:

.. code-block:: bash

    for file in $(find <path/to/package/directory> -name "*.py"); do
        docformatter --in-place "$file"
    done

This may NOT fix all the errors, but it will eliminate some of them.

· Running and Passing 'ament_cppcheck'
""""""""""""""""""""""""""""""""""""""""

The test ``ament_cppcheck`` can be launched by running the next command from your package directory:

.. code-block:: bash

    ament_cppcheck

You may encounter the following error when running the test alone:

.. code-block:: bash

    cppcheck 2.7 has known performance issues and therefore will not be used, set the AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS environment variable to override this.

This can be fixed by setting the ``AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS`` to whatever that evaluates to 'True', just as the error message indicates:

.. code-block:: bash

    export AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=true


.. _development_guide_new_pkg_test_functional:

Code Functional Test
--------------------

In aerostack2 we use googletest (GTest) library to perform unit tests across the packages.
GTest complete documentation about how to write your own unit tests can be found at:
https://github.com/google/googletest

To use GTest, the next line must be added to your ``package.xml``

.. code-block::

    <test_depend>ament_cmake_gtest</test_depend>

In order to compile this tests some lines must be added into a **NEW** ``CMakeLists.txt`` file located in a ``tests/`` folder.

.. code-block:: cmake

    # Tests
    file(GLOB TEST_SOURCE "*_test.cpp")

    if(TEST_SOURCE)
    foreach(TEST_FILE ${TEST_SOURCE})
        get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

        add_executable(${PROJECT_NAME}_${TEST_NAME} ${TEST_FILE})
        ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
        target_link_libraries(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_NAME})
    endforeach()
    endif()

    # GTest
    file(GLOB GTEST_SOURCE "*_gtest.cpp")

    if(GTEST_SOURCE)
    find_package(ament_cmake_gtest REQUIRED)

    foreach(TEST_SOURCE ${GTEST_SOURCE})
        get_filename_component(TEST_NAME ${TEST_SOURCE} NAME_WE)

        ament_add_gtest(${PROJECT_NAME}_${TEST_NAME} ${TEST_SOURCE})
        ament_target_dependencies(${PROJECT_NAME}_${TEST_NAME} ${PROJECT_DEPENDENCIES})
        target_link_libraries(${PROJECT_NAME}_${TEST_NAME} gtest_main ${PROJECT_NAME})
    endforeach()
    endif()

    # Benchmark
    file(GLOB BENCHMARK_SOURCE "*_benchmark.cpp")

    if(BENCHMARK_SOURCE)
    find_package(benchmark REQUIRED)

    foreach(BENCHMARK_FILE ${BENCHMARK_SOURCE})
        get_filename_component(BENCHMARK_NAME ${BENCHMARK_FILE} NAME_WE)

        add_executable(${PROJECT_NAME}_${BENCHMARK_NAME} ${BENCHMARK_FILE})
        target_link_libraries(${PROJECT_NAME}_${BENCHMARK_NAME} ${PROJECT_NAME} benchmark::benchmark)
    endforeach()
    endif()

In order to link and run your functional tests alongside with the code style ones, your package's CMakeLists.txt should follow the next structure:

.. code-block:: cmake

    # Set the minimum required CMake version
    cmake_minimum_required(VERSION 3.5)

    # Set the project name
    set(PROJECT_NAME as2_node_template)
    project(${PROJECT_NAME})

    # Default to C++17 if not set
    if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    endif()

    # Set Release as default build type if not set
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
    endif()

    # find dependencies
    set(PROJECT_DEPENDENCIES
    ament_cmake
    rclcpp
    as2_core
    as2_msgs
    std_msgs
    )

    foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
    find_package(${DEPENDENCY} REQUIRED)
    endforeach()

    # Include necessary directories
    include_directories(
    include
    include/${PROJECT_NAME}
    )

    # Set source files
    set(SOURCE_CPP_FILES
    src/${PROJECT_NAME}_node.cpp
    src/${PROJECT_NAME}.cpp
    )

    # Create the node executable
    add_executable(${PROJECT_NAME}_node ${SOURCE_CPP_FILES})
    ament_target_dependencies(${PROJECT_NAME}_node ${PROJECT_DEPENDENCIES})

    # Create the dynamic library
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
    add_library(${PROJECT_NAME} SHARED ${SOURCE_CPP_FILES})
    ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

    # Set the public include directories for the library
    target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

    # Install the headers
    install(
    DIRECTORY include/
    DESTINATION include
    )

    # Install the node executable
    install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
    )

    # Install the shared library
    install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )

    # Export the libraries
    ament_export_libraries(${PROJECT_NAME})

    # Export the targets
    ament_export_targets(export_${PROJECT_NAME})

    # Export the include directories
    ament_export_include_directories(include)

    # Install the launch directory
    install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
    )

    # Install the config directory
    install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}
    )

    # Build tests if testing is enabled
    if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    file(GLOB_RECURSE EXCLUDE_FILES
        build/*
        install/*
    )
    set(AMENT_LINT_AUTO_FILE_EXCLUDE ${EXCLUDE_FILES})
    ament_lint_auto_find_test_dependencies()

    add_subdirectory(tests)
    endif()

    # Create the ament package
    ament_package()


To run these tests:

.. code-block:: bash

    colcon test

    


Node Template for New Packages
==============================

An example node with the proper structure and all the templates can be found `here <https://github.com/aerostack2/as2_node_template>`_.
