# general python
from typing import Union, Optional

# omniverse imports
import omni
from pxr import UsdGeom, Usd

# library imports
from .core import *

class Prim():
    def __init__(self, prim_path : str):
        self.prim_path = prim_path
        self.visible = True

    @property
    def Prim(self) -> Usd.Prim:
        return stage().GetPrimAtPath(self.prim_path)

    @property
    def exists(self) -> bool:
        return self.Prim.IsValid()
    def __bool__(self):
        return self.exists
    
    def __str__(self) -> str:
        str(self.Prim)

    def toggle(self, force_visibility:Optional[bool]=None):
        self._is_visible = (not self._is_visible) if force_visibility is None else (force_visibility)
        imageable = UsdGeom.Imageable(self.Prim)
        imageable.MakeVisible() if self._is_visible else imageable.MakeInvisible()
    
    def delete(self):
        """Removes this prim from the stage."""
        stage().RemovePrim(self.prim_path)