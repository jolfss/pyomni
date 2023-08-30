# general python
from typing import Tuple
import numpy as np

# library imports
from .core import stage
from .prims import *

class Cube(Scaleable, Colorable):
    """Represents a Cube prim, inheriting Imageable.

    Attributes:
        prim_path (str): The USD prim path where the cube is defined.
        color (float,float,float): The r,g,b color of the cube. 
            NOTE: Not mutable, but assignable
        scale (float,float,float): The x,y,z scales of the cube (technically a rectangular prism)
            NOTE: Not mutable, but assignable
        """
    
    def __init__(self, prim_path : str):
        """Args:
            prim_path (str): The USD prim path where the cube should be defined."""
        Scaleable.__init__(self, prim_path)
        Colorable.__init__(self, prim_path)
        UsdGeom.Cube.Define(stage(), prim_path)
        self._displayColorAttr = self.UsdPrim.GetAttribute('primvars:displayColor')
        self.scale # Calling the getter to immediately satisfy the invariant and fill the self._scaleOp attribute.



