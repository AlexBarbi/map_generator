"""Puts scripts/ on the path: the tests import the scripts as modules."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
