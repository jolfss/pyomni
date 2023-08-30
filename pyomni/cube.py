# general python
from typing import Tuple
import numpy as np

# omniverse imports
from pxr import UsdGeom, Gf

# library imports
from .core import stage
from .prims import Imageable

class Cube(Imageable):
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
        super().__init__(prim_path)
        UsdGeom.Cube.Define(stage(), prim_path)
        self._displayColorAttr = self.UsdPrim.GetAttribute('primvars:displayColor')
        self._scaleOp = None

    @property
    def color(self) -> np.ndarray:
        return np.copy(self._displayColorAttr.Get())
    @color.setter
    def color(self, color : np.ndarray):
        self._displayColorAttr.Set([(Gf.Vec3f(*color))])

    @property
    def scale(self) -> np.ndarray:
        xformable = UsdGeom.Xformable(self.UsdPrim)
        scaleOp = None # Checks to make sure scale op hasn't been removed/changed reference.
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                scaleOp = op
                break
        if not scaleOp:
            scaleOp = xformable.AddScaleOp()
        self._scaleOp = scaleOp
        return np.copy(scaleOp.Get())
    @scale.setter
    def scale(self, new_scales:np.ndarray):
        pass # TODO: Fix cube scale from ndarray
    


