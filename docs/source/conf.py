import os
import sys

# Add workspace root
sys.path.insert(0, os.path.abspath("../.."))

# Add ROS Python packages (install space)
ros_install = os.environ.get("AMENT_PREFIX_PATH")
if ros_install:
    for path in ros_install.split(":"):
        python_path = os.path.join(path, "lib", "python3.12", "site-packages")
        if os.path.isdir(python_path):
            sys.path.insert(0, python_path)


# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'Mensabot'
copyright = '2026, Mensabot_Admin'
author = 'Mensabot_Admin'
release = '0.1.0'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.napoleon",
    "sphinx.ext.viewcode",
    "myst_parser",
    "sphinx_autodoc_typehints",
]


templates_path = ['_templates']
exclude_patterns = []



# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "sphinx_rtd_theme"

html_static_path = ['_static']


autodoc_mock_imports = [
    "rclpy",
    "tf2_ros",
    "tf2_geometry_msgs",
    "geometry_msgs",
    "nav_msgs",
    "visualization_msgs",
    "builtin_interfaces",
    "robot_msgs",
    "numpy",
]
