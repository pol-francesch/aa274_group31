############################################################
# ExtractLines.py
#
# This script reads in range data from a csv file, and
# implements a split-and-merge to extract meaningful lines
# in the environment.
############################################################

# Imports
import numpy as np
import os
import pickle

# Not particularly strong obfuscation, but the Honor Code should otherwise be sufficient.
with open(os.path.join(os.path.dirname(__file__), "ExtractLines.pickle"), "rb") as f:
    function_dict = pickle.load(f)

locals().update(function_dict)
