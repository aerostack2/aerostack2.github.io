from sphinx.locale import _
import os
import sys
import re
sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath(
    './_07_python_api/temp_ws/src/as2_python_api/as2_python_api'))

# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
# import os
# import sys
# sys.path.insert(0, os.path.abspath('.'))


# -- Project information -----------------------------------------------------

project = 'Aerostack2'
copyright = '2022-2023, Universidad Politécnica de Madrid'
author = 'CVAR'
slug = re.sub(r'\W+', '-', project.lower())

# The full version, including alpha/beta/rc tags
release = '1.0'


# -- General configuration ---------------------------------------------------

breathe_projects = {
    "as2_core": "./_09_development/_api_documentation/as2_core/doxygen/xml"
}

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'sphinx.ext.intersphinx',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosummary',
    'sphinx.ext.mathjax',
    'sphinx.ext.viewcode',
    'sphinx_rtd_theme',
    'sphinx_copybutton',
    'breathe',
]
# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

autodoc_mock_imports = ['as2_msgs', 'geographic_msgs',
                        'as2_motion_reference_handlers',
                        'PySimpleGUI', 'pymap3d', 'pydantic']

autodoc_default_options = {'autosummary': False}

# -- Options for HTML output -------------------------------------------------

html_context = {
    "display_github": True,  # Integrate GitHub
    "github_user": "aerostack2",  # Username
    "github_repo": "aerostack2.github.io",  # Repo name
    "github_version": "main",  # Version
    "conf_py_path": "/docs/",  # Path in the checkout to the docs root
}

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
try:
    import sphinx_rtd_theme
except ImportError:
    html_theme = 'alabaster'
    # This is required for the alabaster theme
    # refs: http://alabaster.readthedocs.io/en/latest/installation.html#sidebars
    html_sidebars = {
        '**': [
            'relations.html',  # needs 'show_related': True theme option to display
            'searchbox.html',
        ]
    }
    sys.stderr.write(
        'Warning: sphinx_rtd_theme missing. Use pip to install it.\n')
else:
    html_theme = "sphinx_rtd_theme"
    html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
    html_theme_options = {
        'canonical_url': '',
        'analytics_id': '',
        'logo_only': False,
        'logo': "_images/logo.png",
        'display_version': True,
        'prev_next_buttons_location': 'None',
        'style_nav_header_background': 'blue',
        # Toc options
        'collapse_navigation': False,
        'sticky_navigation': True,
        'navigation_depth': 4,
    }

html_theme_path = ['_themes']

html_theme = 'sphinx_rtd_theme'
html_logo = "_images/logo.png"
logo = "_images/logo.png"
# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
# html_static_path = ['_themes/sphinx_rtd_theme/static']

html_css_files = [
    'css/config.css',
]

latex_documents = [
    ('index', '{0}.tex'.format(slug), project, author, 'manual'),
]

man_pages = [
    ('index', slug, project, [author], 1)
]

texinfo_documents = [
    ('index', slug, project, author, slug, project, 'Miscellaneous'),
]


def setup(app):
    from sphinx.domains.python import PyField
    from sphinx.util.docfields import Field

    app.add_object_type(
        'confval',
        'confval',
        objname='configuration value',
        indextemplate='pair: %s; configuration value',
        doc_field_types=[
            PyField(
                'type',
                label=_('Type'),
                has_arg=False,
                names=('type',),
                bodyrolename='class'
            ),
            Field(
                'default',
                label=_('Default'),
                has_arg=False,
                names=('default',),
            ),
        ]
    )
