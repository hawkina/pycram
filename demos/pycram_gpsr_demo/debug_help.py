import sys
import importlib
import logging

# Set up logging to display debug messages
logging.basicConfig(level=logging.DEBUG, format='%(message)s')

# Store the original import function
original_import = __builtins__.__import__


def log_import(name, globals=None, locals=None, fromlist=(), level=0):
    logging.debug(f"Importing {name}")
    return original_import(name, globals, locals, fromlist, level)

# Replace the built-in import function with the logging version
# __builtins__.__import__ = log_import
