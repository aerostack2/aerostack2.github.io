# CONTRIBUTION GUIDE (DEPRECATED)(WIP)

This repository contain the source files to build Aerostack2's documentation from docs folder. 
In order to build the project, just push the changes to docs and github actions will automatically push
the built html files in branch **gh-pages**.

## Adding a python module to docs

To add a python module to the web, these are the steps:
- Configure the python package so that Sphinx can build the html files for it.  To do so, you should create a Sphinx project so that the folder docs/ contains the following files:
	- make.bat
	- Makefile
	- Folder sources/
- index.rst and conf.py will be placed inside the sources/ folder, however, we don't need the conf.py as the github action will take care of that. With index.rst, you can start referencing documents and creating a documentation structure. 
- Inside the docs/ folder, in case you want documentation to be generated automatically, create a module_name.rst with the following: 
```
	.. automodule:: module_name
   :members:
   :undoc-members:
   :show-inheritance:
```
This will make autodoc extension generate all the .rst files from the source python files in the directory below.
- After this, you can add the following line to index.rst in order to make the reference:
```
.. toctree::
   :maxdepth: 1
   :caption: Content:

   module_name
```

- Add a checkout action to the workflow of this repository. This action will bring the code into a folder inside the the docker OS beeing executed in the workflow. Example:
``` 
    - uses: actions/checkout@master
      with:
        repository: aerostack2/python_interface
        ref: main
        path: python_interface
        fetch-depth: 0 # otherwise, you will failed to push refs to dest repo
```
- In this repository https://github.com/aerostack2/aerostack2.github.io inside the docs folder, add the following import 
```python
sys.path.insert(0, os.path.abspath('./_category/temp_ws/src/package_name/package_name'))
```
Where _category is the section where we want to add the documentation. For example, if I would like to add the python package docs to the user section, _category would be _user.

- Inside the _category folder, there should be an index.rst file. Edit this file and add wherever you want your doc to be located inside the section, the following:
```
.. toctree::
   :maxdepth: 2

   temp_ws/src/package_name/docs/source/index.rst
```
temp_ws/src is the directory inside the docker OS filesystem where the python package will be ros-built. This is necessary so that autodoc can automatically generate the documentation from the python source files. 

- **IMPORTANT**: If there is a missing dependency, even if the ros-build phase is completed, autodoc won't generate the documentation. We can make autodoc ignore these dependencies by adding these to the rule:
```
autodoc_mock_imports = ['as2_msgs', 'geographic_msgs', 'motion_reference_handlers']
```
inside conf.py file. This is to be improved in later versions.
