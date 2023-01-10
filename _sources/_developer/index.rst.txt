Developer guide
===============

File style guide
################

Every file should start with the Licence text, which looks like this:

.. code-block:: c++

    /*********************************************************************************************
    *  \file       [file name]
    *  \brief      [brief description of the file]
    *  \authors    [name of the author]
    *  \copyright  Copyright (c) 2021 Universidad Politécnica de Madrid
    *              All Rights Reserved
    *
    * Redistribution and use in source and binary forms, with or without
    * modification, are permitted provided that the following conditions are met:
    * 
    * 1. Redistributions of source code must retain the above copyright notice,
    *    this list of conditions and the following disclaimer.
    * 2. Redistributions in binary form must reproduce the above copyright notice,
    *    this list of conditions and the following disclaimer in the documentation
    *    and/or other materials provided with the distribution.
    * 3. Neither the name of the copyright holder nor the names of its contributors
    *    may be used to endorse or promote products derived from this software
    *    without specific prior written permission.
    * 
    * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
    * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
    * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
    * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
    * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
    * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
    * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
    * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
    * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    **********************************************************************************************/

Code should follows the `google c++ style guide <https://google.github.io/styleguide/cppguide.html>`_.

Also, documentation will be generate using `Doxygen <https://www.doxygen.nl/manual/docblocks.html>`_. Therefore, header files should include a comment over every definition in order to generate the documentation properly. The comments made in the current nodes are writing using Javadoc style.

Plese, do not use std::cout or similar functions to show messages in command line. Instead, use logging macros like RCLCPP_INFO from ROS logging library.

Developing a new Package
########################

Style tests
-----------

A Cpp style check must be done in order to use the same style through all the packages.
Each package must have the same ``.clang-format`` `file <https://github.com/aerostack2/as2_node_template/blob/main/.clang-format>`_.

For configuring style tests, this must be added on the ``CMakeList.txt`` 

.. code-block:: cmake

    if(BUILD_TESTING)

    find_package(ament_cmake_cppcheck REQUIRED)
    find_package(ament_cmake_clang_format REQUIRED)
    
    ament_cppcheck(src/ include/ tests/)
    ament_clang_format(src/ include/ tests/ --config ${CMAKE_CURRENT_SOURCE_DIR}/.clang-format)

    endif()

Also, these packages must be added to the ``package.xml``

.. code-block::

    <test_depend>ament_cmake_clang_format</test_depend>
    <test_depend>ament_cmake_cppcheck</test_depend>


if ament clang format test fails you can autoformat the code using :

.. code-block:: bash

    $ ament_clang_format ./src/ ./include/ ./tests/ --reformat --config .clang-format

Functional tests
----------------

In aerostack2 we use googletest (GTest) library to perform unit tests across the packages.
GTest complete documentation about how to write your own unit tests can be found at:
https://github.com/google/googletest

In order to compile this tests some lines must be added into a **NEW** ``CMakeLists.txt`` file located in a ``tests/`` folder.

.. code-block:: cmake

    # Add gtest dependencies and install them if they are not already installed

    find_package(gtest QUIET)
    if (${Gtest_FOUND})
    MESSAGE(STATUS "Found Gtest.")
    else (${Gtest_FOUND})
    MESSAGE(STATUS "Could not locate Gtest.")
    include(FetchContent)
    FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/609281088cfefc76f9d0ce82e1ff6c30cc3591e5.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    endif(${Gtest_FOUND})

    include(GoogleTest)

    enable_testing()
    # find all *.cpp files in the tests directory

    file(GLOB TEST_SOURCES tests/*.cpp )

    # create a test executable for each test file
    foreach(TEST_SOURCE ${TEST_SOURCES})

    get_filename_component(_src_filename ${TEST_SOURCE} NAME)
    string(LENGTH ${_src_filename} name_length)
    math(EXPR final_length  "${name_length}-4") # remove .cpp of the name
    string(SUBSTRING ${_src_filename} 0 ${final_length} TEST_NAME)
    
    add_executable(${TEST_NAME}_test ${TEST_SOURCE})
    ament_target_dependencies(${TEST_NAME}_test  ${PROJECT_DEPENDENCIES})
    target_link_libraries(${TEST_NAME}_test gtest_main ${PROJECT_NAME})

    # add the test executable to the list of executables to build
    gtest_discover_tests(${TEST_NAME}_test)

    endforeach()

In order to link this ``./tests/CMakeLists.txt`` file into the ``CMakeLists.txt`` file of the package, the following line must be added:

.. code-block:: cmake

    if(BUILD_TESTING)
    # all the other tests
    include(./tests/CMakeLists.txt)
    endif()


Run tests
---------

To run these tests:

.. code-block:: bash

    $ colcon test 

Package architecture
####################

Every new package should have the following architecture:

.. code-block::

    /package_name
    /include
        /package_name
        header_files(.hpp)
    /launch
    /src
        implementation_files(.cpp)
        main_files(.cpp)
    /tests
        CMakeLists.txt
        test_files(.cpp)
    CmakeLists.txt
    package.xml
    Readme.md
    .gitignore