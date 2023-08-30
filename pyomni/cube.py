# general python
from typing import Tuple

# omniverse imports
from pxr import UsdGeom, Gf

# library imports
from .core import stage
from .prims import Visible

class Cube(Visible):
    def __init__(self, prim_path : str):
        super().__init__(prim_path)
        UsdGeom.Cube.Define(stage(), prim_path)
        self._displayColorAttr = self.UsdPrim.GetAttribute('primvars:displayColor')


    @property
    def color(self):
        r,g,b = self._displayColorAttr.Get()
        return r,g,b
    @color.setter
    def color(self, color : Tuple[float,float,float]):
        self._displayColorAttr.Set([(Gf.Vec3f(*color))])

    @property
    def scale(self):
        xformable = UsdGeom.Xformable(self.UsdPrim)
        scaleOp = None # check on if prim already has scale op
        for op in xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeScale:
                scaleOp = op
                break
        if not scaleOp:
            scaleOp = xformable.AddScaleOp()
        return scaleOp
    @scale.setter
    def scale(self, new_scales:Tuple[float,float,float]):
        self.scale.Set(value=new_scales)

    


