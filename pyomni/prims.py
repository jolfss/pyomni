# general python
from typing import Union, Optional
import numpy as np

# omniverse imports
import omni
from pxr import UsdGeom, Usd, Gf

# library imports
from .core import *


class Primitive():
    """Represents a generic primitive in the current Usd context.
    Attributes:
        UsdPrim (Usd.Prim): The object containing the entire interface exposed to Python. 
        exists (bool): True if [prim_path] is occupied.
    Methods:
        delete(self): Removes this prim from the stage."""
    def __init__(self, prim_path : str):
        self.prim_path = prim_path

    @property
    def UsdPrim(self) -> Usd.Prim:
        return stage().GetPrimAtPath(self.prim_path)

    @property
    def exists(self) -> bool:
        return self.UsdPrim.IsValid()
    def __bool__(self):
        return self.exists
    
    def __str__(self) -> str:
        return str(self.UsdPrim)

    def delete(self):
        """Removes this prim from the stage."""
        stage().RemovePrim(self.prim_path)

class Imageable(Primitive):
    """Represents a prim that can be rendered or visualized.
    Attributes:
        visible (bool): Whether or not this geometry is visible.
    Methods:
        toggle(self, force_visibility:bool option): Toggles the prim's visibility."""
    def __init__(self, prim_path):
            super().__init__(prim_path)
            self._is_visible = True 
        
    def toggle(self, force_visibility: Optional[bool] = None):
        """Flips the visibility of a primitive unless an override is specified."""
        self._is_visible = not self.visible if force_visibility is None else force_visibility
        imageable = UsdGeom.Imageable(self.UsdPrim)
        imageable.MakeVisible() if self.visible else imageable.MakeInvisible()

    @property
    def visible(self) -> bool:
        "The current visibility state of the imageable primitive."
        return self._is_visible
    @visible.setter
    def visible(self, value: bool):
        self.toggle(force_visibility=value)

#----------------#
#   attributes   #
#----------------#

class Colorable(Imageable):
    """Represents a prim that can be rendered or visualized.
    Attribute:
        color (ndarray): (3,) float array [r,g,b] with r,g,b in [0,1]."""
    def __init__(self, prim_path):
        super().__init__(prim_path)

    @property
    def color(self) -> np.ndarray:
        """A (3,) array containing the [r,g,b] color of the cube as floats in [0,1]."""
        return np.copy(self._displayColorAttr.Get())
    @color.setter
    def color(self, color : np.ndarray):
        self._displayColorAttr.Set([(Gf.Vec3f(*color))])

class Scaleable(Imageable):
    """Represents a prim that can be scaled.
    Attribute:
        scale (np.ndarray): (3,) float array [x,y,z] for the scale of the cube in each dimension."""
    def __init__(self, prim_path):
        super().__init__(prim_path)
        
    @property
    def scale(self) -> np.ndarray:
        """A (3,) array containing the [x,y,z] scale of the primitive."""
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
        self._scaleOp.Set(new_scales)

class Rotatable(Imageable):
    # TODO: quaternion representation thing.
    """Represents a prim that can be rotated.
    Attribute:
        rotation (np.ndarray): (3,) float array [x,y,z] for the scale of the cube in each dimension."""
    def __init__(self, prim_path):
        super().__init__(prim_path)
        
    @property
    def scale(self) -> np.ndarray:
        """A (3,) array containing the [x,y,z] scale of the primitive."""
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
        self._scaleOp.Set(new_scales)
