# -*- coding: utf-8 -*-

import os
import sys
import xml.etree.ElementTree as etree

sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)))

extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.doctest',
    'sphinx.ext.viewcode',
]

source_suffix = '.rst'
master_doc = 'index'

project = u'jackal_tutorials'
copyright = u'2015, Clearpath Robotics'

# Get version number from package.xml.
tree = etree.parse('../package.xml')
version = tree.find("version").text
release = version

html_theme = 'nature'
htmlhelp_basename = 'jackal_tutorialsdoc'
templates_path = ['./templates']
html_static_path = ['./static']

rst_prolog = """
.. |ros_distro| replace:: indigo
.. |ubuntu_distro| replace:: trusty
"""
#.. ubuntu_distro: trusty

