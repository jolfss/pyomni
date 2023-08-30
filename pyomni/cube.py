# library imports
from .core import stage
from .prims import *

class Cube(Scaleable, Colorable):
    """Represents a Cube prim."""
    
    def __init__(self, prim_path : str):
        """Creates a cube within the scene.
        Args:
            prim_path (str): The USD prim path where the cube should be defined."""
        Scaleable.__init__(self, prim_path)
        Colorable.__init__(self, prim_path)
        UsdGeom.Cube.Define(stage(), prim_path)
        self._displayColorAttr = self.UsdPrim.GetAttribute('primvars:displayColor')
        self.scale # Calling the getter to immediately satisfy the invariant and fill the self._scaleOp attribute.



