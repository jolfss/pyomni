# omniverse imports
import omni

# library imports
from .core import *

def context():
    """Fetches the current Usd context from a running Omniverse instance."""
    return omni.usd.get_context()

def stage():
    """Fetches the currently active stage from a running Omniverse instance."""
    return omni.usd.get_context().get_stage()