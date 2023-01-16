# Auto building in VS Code
1. Install requirements
```
pip install -r requirements.txt
```

2. Install VS Code Extension:

    *reStructuredText* by LeXtudio

3. Install esbonio server
```
pip install esbonio==0.15.0
```

4. VS Code configuration file
```
{
    // rst
    "esbonio.sphinx.buildDir" : "${workspaceFolder}/_build/html",
    "esbonio.sphinx.confDir"  : "${workspaceFolder}/docs",
    "esbonio.sphinx.srcDir"   : "${workspaceFolder}/docs"
}
```

Make sure that your project settings doesn't overwrite the `esbonio.sphinx.confDir` configuration.

# Build manually
```
sphinx-build -b html docs/ docs/build/
```