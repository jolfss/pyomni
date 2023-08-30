# general python
from typing import Union, Optional

# omniverse imports
import omni
from pxr import UsdGeom, Usd

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
        UsdPrim (Usd.Prim): The object containing the entire interface exposed to Python. 
        exists (bool): True if [prim_path] is occupied.
    Methods:
        toggle(self, force_visibility:bool option): Toggles the prim's visibility.
        delete(self): Removes this prim from the stage."""
    def __init__(self, prim_path):
            super().__init__(prim_path)
            self._is_visible = True 
        
    def toggle(self, force_visibility: Optional[bool] = None):
        self._is_visible = not self.visible if force_visibility is None else force_visibility
        imageable = UsdGeom.Imageable(self.UsdPrim)
        imageable.MakeVisible() if self.visible else imageable.MakeInvisible()

    @property
    def visible(self):
        return self._is_visible

    @visible.setter
    def visible(self, value: bool):
        self.toggle(force_visibility=value)

