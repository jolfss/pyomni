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
        color (np.ndarray): For textureless geometry, the base color of the object.
        position (np.ndarray): 
        rotation (np.ndarray): 
        scale (np.ndarray): 
    Methods:
        toggle(self, force_visibility:bool option): Toggles the prim's visibility."""
    def __init__(self, prim_path):
            super().__init__(prim_path)
            self._is_visible = True 

    @property
    def color(self) -> np.ndarray:
        """A (3,) array containing the [r,g,b] color of the cube as floats in [0,1]."""
        return np.copy(self._displayColorAttr.Get())
    @color.setter
    def color(self, color : np.ndarray):
        self._displayColorAttr.Set([(Gf.Vec3f(*color))])

    @property
    def position(self) -> np.ndarray:
        """A (3,) array containing the [x, y, z] position of the primitive."""
        translateOp = None
        for op in self._xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                translateOp = op
                break
        if not translateOp:
            translateOp = self._xformable.AddTranslateOp()
        self._translateOp = translateOp
        return np.copy(translateOp.Get())

    @position.setter
    def position(self, new_position: np.ndarray):
        self._translateOp.Set(new_position)

    @property
    def rotation(self) -> np.ndarray:
        """A (4,) array containing the quaternion [x, y, z, w] rotation of the primitive."""
        rotateOp = None
        for op in self._xformable.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:  # Assuming using Euler angles
                rotateOp = op
                break
        if not rotateOp:
            rotateOp = self._xformable.AddRotateXYZOp()
        self._rotateOp = rotateOp
        return np.copy(rotateOp.Get())

    @rotation.setter
    def rotation(self, new_rotation: np.ndarray):
        self._rotateOp.Set(new_rotation)

    @property
    def scale(self) -> np.ndarray:
        """A (3,) array containing the [x,y,z] scale of the primitive."""
        xformable = UsdGeom.Xformable(self.UsdPrim)
        scaleOp = None
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
    
    def toggle(self, force_visibility: Optional[bool] = None):
        """Flips the visibility of a primitive unless an override is specified."""
        self._is_visible = not self.visible if force_visibility is None else force_visibility
        imageable = UsdGeom.Imageable(self.UsdPrim)
        imageable.MakeVisible() if self.visible else imageable.MakeInvisible()