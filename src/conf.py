import sys
import os
from unittest import mock



# TODO: install doxyrest properly
# Needed by Doxygen -> rST generated docs
# https://github.com/vovkos/doxyrest
sys.path.insert(1, os.path.abspath('../.ci/doxyrest/sphinx'))

sys.path.append('../../lib')

#from theme.conf import *
extensions = [  'sphinx.ext.viewcode',
                'sphinx.ext.autodoc',
                'sphinxcontrib.httpdomain',
                'sphinxcontrib.autohttp.flask',
                'sphinxcontrib.images',
                'sphinx.ext.coverage',
                'sphinx.ext.autosummary',
                'sphinx.ext.todo',
                'recommonmark',
                'sphinx_copybutton',
                'doxyrest',
                'cpplexer']

authors = u'TBD'
latex_authors = authors.replace(',', r'\and')
project = title = u'HBP Neurorobotics Platform'
basename = u'HBPNeuroroboticsPlatformDocumentation'
description = u'HBP Neurorobotics Platform user manual'
copyright = u'2023, Human Brain Project'

# -- General configuration ------------------------------------------------

# This values are to be overwritten by projects
version = "undefined"
release = "undefined"


images_config = {
    "override_image_directive": True
}

numfig = True

todo_include_todos = False
todo_emit_warnings = True

show_authors = False

#ADDED BY FOM
html_static_path = ['_static']

html_css_files = [
    'css/hbpdoc.css',
]

html_js_files = [
    'matomo/mt.js'
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# The suffix of source filenames.
source_suffix = {
    '.rst': 'restructuredtext',
    '.md': 'markdown',
}

# The master toctree document.
master_doc = 'index'

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
exclude_patterns = [
    '_build', 
    'nrp-core/page_index.rst'
]

# the following modules are part of CLE and should be mocked
autodoc_mock_imports = ['nrp_core']

# The name of the Pygments (syntax highlighting) style to use.
pygments_style = 'sphinx'


# -- Options for HTML output ----------------------------------------------
html_theme = "sphinx_rtd_theme" #added by FOM
html_show_sphinx = False

# Theme options are theme-specific and customize the look and feel of a theme
# further.  For a list of options available for each theme, see the
# documentation.
html_theme_options = { 
    }


# Custom sidebar templates, maps document names to template names.
html_sidebars = {
   '**': ['globaltoc.html', 'sourcelink.html', 'searchbox.html'],
   'using/windows': ['windowssidebar.html', 'searchbox.html']
}


# -- Options for manual page output ---------------------------------------

# One entry per manual page. List of tuples
# (source start file, name, description, authors, manual section).
man_pages = [
    ('index', basename.lower(), title,
     [authors], 1)
]

# If true, show URL addresses after external links.
#man_show_urls = False


# -- Options for Texinfo output -------------------------------------------

# Grouping the document tree into Texinfo files. List of tuples
# (source start file, target name, title, author,
#  dir menu entry, description, category)
texinfo_documents = [
  ('index', basename, title,
   authors, basename, title,
   'Miscellaneous'),
]

# Mock for autoflask
MOCK_MODULES = ['nrp_core', 'nrp_core.client']
for mod_name in MOCK_MODULES:
    sys.modules[mod_name] = mock.Mock()
