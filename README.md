# CONTRIBUTION GUIDE 

This repository contain the source files to build Aerostack2's documentation from docs folder. 
In order to build the project, just push the changes to docs and github actions will automatically push
the built html files in branch **gh-pages**.

## Adding a python package to docs

To add a python module to the web, follow these steps:
- Configure the python package so that Sphinx can build the .html files for it.  To do so, you have to [create a Sphinx project](https://www.sphinx-doc.org/en/master/tutorial/getting-started.html#setting-up-your-project-and-development-environment) **Do not build** so that the folder package/docs/ contains the following files:
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

- Add the package name to the input aerostack2_modules to the workflow of this repository. This action will bring the package into a _folder inside the the docker OS being executed in the workflow. Example if we want to add 'my_python_package':
``` 
    - name: Build and Commit
      uses: aerostack2/pages@v2
      with:
        aerostack2_modules: package1, package2, my_python_package
```
- Now choose what category you want your file to be included in. Inside this repository https://github.com/aerostack2/aerostack2.github.io in the _category folder, there should be an index.rst file. If you want to create a new category, just create folder with name: _category, place an index.rst file, and reference it in the main index.rst file in the docs/ folder of this repository. Edit this _category/index.rst and add the following:
```
.. toctree::
   :maxdepth: 2

   temp_ws/src/package_name/docs/source/index.rst
```
**A package can only be placed in one _category**. Also, temp_ws/src is the directory inside the docker OS filesystem where the python package will be ros-built. This is necessary so that autodoc can automatically generate the documentation from the python source files.
-  inside this repo's docs/ folder there is a conf.py file. Add the following line to it:
```python
sys.path.insert(0, os.path.abspath('./_category/temp_ws/src/package_name/package_name'))
```
Where _category is the section where we want to add the documentation.

**IMPORTANT**: If there is a missing dependency, even if the ros-build phase is completed, autodoc won't generate the documentation. We can make autodoc ignore these dependencies by adding these dependencies to the rule inside this repo's conf.py file. For example:
```
autodoc_mock_imports = ['as2_msgs', 'geographic_msgs', 'motion_reference_handlers']
```
This is to be improved in later versions.

## Adding a C++ project to docs

To add a C++ module to the web, follow these steps:
- Grab from this repository https://github.com/aerostack2/aerostack2.github.io the doxygen.dox file inside the doxygen_template/ folder and put it into the C++ project root folder. 
- Modify first two lines of this file, substituting as2_package for the name of your C++ project.
```
  PROJECT_NAME = as2_cpp_package
  INPUT = ./include/as2_cpp_package ./src
  ```
  - Create docs/ folder inside the root folder of your package and place an index.rst with the following:
  ```
	  as2_package
	  ===========
	.. doxygenindex::
	   :project: as2_package
  ```
  - Add the package name to the input aerostack2_modules to the workflow of this repository. This action will bring the package into a _folder inside the the docker OS being executed in the workflow. Example if we want to add 'as2_cpp_package':
``` 
    - name: Build and Commit
      uses: aerostack2/pages@v2
      with:
        aerostack2_modules: package1, package2, as2_cpp_package
```
- Now choose what category you want your file to be included in. Inside this repository https://github.com/aerostack2/aerostack2.github.io in the _category folder, there should be an index.rst file. If you want to create a new category, just create folder with name: _category, place an index.rst file, and reference it in the main index.rst file in the docs/ folder of this repository. Edit this _category/index.rst file and add the following:
```
.. toctree::
   :maxdepth: 2

   as2_cpp_package/docs/index.rst
```
**A package can only be placed in one _category**.
  
- Finally, inside this repo's docs/ folder there is a conf.py file. Edit dictionary 'breathe_projects', and in case we want to add 'as2_cpp_package' in _category/ folder, add an entry as the following example:
```
breathe_projects = {
    "other_cpp_package": "./_other_category/other_cpp_package/doxygen/xml",
    "as2_cpp_package": "./_category/as2_cpp_package/doxygen/xml"
}
```
