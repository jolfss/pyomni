from typing import Union

# omniverse imports
import omni

# library imports
from .core import *

def context():
    return omni.usd.get_context()

def stage():
    return omni.usd.get_context().get_stage()

#-----------#
#   debug   #
#-----------#
def warn(message):
    print(F"[pyomni] Warning: {message}")