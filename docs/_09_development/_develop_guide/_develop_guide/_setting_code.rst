.. _development_guide_code:

-----------------------------
Setting Up Visual Studio Code
-----------------------------

A set of tools can be configured on Visual Studio Code to help adjusting your code to the Aerostack2 Code Style guidelines during the development proccess.

.. _development_guide_code_extensions:

Install required Extensions
===========================

Install the following required extensions from the 'Extensions' tab in VSCode:

* ``zachflower.uncrustify``
* ``mine.cpplint``
* ``ms-python.autopep8``
* ``ms-python.pylint``


.. _development_guide_code_settings:

Modify 'settings.json'
======================

Open your ``settings.json`` file from VSCode and add the following content to it:

.. code-block:: json

    // cpp
    "[cpp]": {
        "editor.defaultFormatter": "zachflower.uncrustify",
        "editor.codeActionsOnSave": {
            "source.fixAll": "explicit"
        },
        "editor.formatOnSave": true
    },
    "uncrustify.configPath.linux": "<path-to>/.uncrustify.cfg",

    // cpplint
    "cpplint.cpplintPath": "<path-to-bin>/cpplint",
    "cpplint.lineLength": 100,
    "cpplint.verbose": 0,
    "cpplint.filters": [
        // header guards are user defined
        "-build/header_guard",
        // we do allow C++11
        "-build/c++11",
        // we consider passing non-const references to be ok
        "-runtime/references",
        // we wrap open curly braces for namespaces, classes and functions
        "-whitespace/braces",
        // we don't indent keywords like public, protected and private with one space
        "-whitespace/indent",
        // we allow closing parenthesis to be on the next line
        "-whitespace/parens",
        // we allow the developer to decide about whitespace after a semicolon
        "-whitespace/semicolon",
    ],

    // python
    "[python]": {
        "editor.defaultFormatter": "ms-python.autopep8",
        "editor.formatOnSave": true,
    },
    "autopep8.args": ["--max-line-length=99", "--ignore=''"],

    // ROS2 uses as a linter flake8==4.0.1 (apt installation). Pip installation should be 
    // at the same version to avoid errors. pycodestyle==2.8.0 is also required.
    // vscode flake8 extension requires at least flake8 v5.0.0. Pylint extension is used 
    // instead with identical configuration.
    "pylint.args": ["--disable=B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202",
                    "--max-line-length=99", "--import-order-style=google", "--show-source=true",
                    "--statistics=true"],

.. _development_guide_code_config_variables:

Set Configuration Variables
===========================

Create '.uncrustify.config' file
--------------------------------

To set the 'uncrustify.configPath.linux' variable, go to any directory and create a file with the name ``.uncrustify.config`` and paste in it the content of the `following file <https://github.com/ament/ament_lint/blob/foxy/ament_uncrustify/ament_uncrustify/configuration/ament_code_style.cfg>`_.

As a recommendation, the file can be created in the 'Code' folder that should be placed in the '.config' directory (~/.config/Code).

Copy the path to the created file to the 'uncrustify.configPath.linux' variable in your 'settings.json' file.

Install and Set 'cpplint'
-------------------------

Make sure you have installed ``cpplint``. If not, install it by running:

.. code-block:: bash

    pip install --user cpplint

Once it is installed, copy the path to the package binaries to the variable 'cpplint.cpplintPath' in your 'settings.json' file.