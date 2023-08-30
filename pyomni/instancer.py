# general python imports
from typing import List, Optional
import numpy as np

# omniverse imports
from pxr import UsdGeom, Gf, Vt, Usd, Sdf   

# library imports
from .core import *
from .prims import *

class Instancer(Imageable):
    """Python wrapper class for the UsdGeom.PointInstancer.
    Attributes:
        positions (ndarray): (N,3) array of positions, 
        protoindices (ndarray): (N) array that specifies which prim should show up at each position.
            i.e., each element in [protoindices] is the index of the corresponding prim in [targets].
    Methods:
        get_targets() -> List: 
        add_target(target : Imageable): Appends an imageable target to the list of prims to display and returns the
            index which should be used to reference it as a protoindex.
        remove_target(target : Imageable): Removes a target from the list of registered targets.
            NOTE: This will decrement the protoindices of all targets which have been added after the removal target.
        """
    def __init__(self, prim_path : str):
        super().__init__(prim_path)
        self._PointInstancer = UsdGeom.PointInstancer.Define(stage(),self.prim_path)
        self._PositionsAttr = self._PointInstancer.GetPositionsAttr()
        self._ProtoIndicesAttr = self._PointInstancer.GetProtoIndicesAttr()
        self._PrototypesRel = self._PointInstancer.GetPrototypesRel()

        #Initialize
        self._PositionsAttr.Set(Vt.Vec3fArray(1))
        self._ProtoIndicesAttr.Set(Vt.IntArray(1))

    @property
    def positions(self) -> np.ndarray:
        return np.copy(self._PositionsAttr.Get())
    @positions.setter
    def positions(self, np_array : np.ndarray):
       self._PositionsAttr.Set(Vt.Vec3fArray.FromNumpy(np_array))
    
    @property
    def protoindices(self) -> np.ndarray:
        return np.copy(self._ProtoIndicesAttr.Get())
    @protoindices.setter
    def protoindices(self, new_protoindices : np.ndarray):
        min_protoindex = new_protoindices.min()
        max_protoindex = new_protoindices.max()
        if min_protoindex < 0:
            warn(F"Instancer recieved index that does not map to a target {min_protoindex}.")
        if max_protoindex >= len(self.get_targets()):
            warn(F"Instancer recieved and index greater than any existing target {max_protoindex}.")
        self._ProtoIndicesAttr.Set(Vt.IntArray.FromNumpy(new_protoindices))
    
    def num_targets(self):
        return len(self._PrototypesRel.GetTargets())

    def get_targets(self) -> List[Imageable]:
        lst = []
        for t in self._PrototypesRel.GetTargets():
            lst.append(Imageable(t.pathString))
        return lst
    
    def add_target(self, target : Imageable):
        self._PrototypesRel.AddTarget(target.prim_path)
        return self.num_targets() - 1

    def remove_target(self, target : Imageable):
        self._PrototypesRel.RemoveTarget(target.prim_path)

    def clear_placements(self):
        self._ProtoIndicesAttr.Clear()
        self._PositionsAttr.Clear()
    
    def clear_targets(self):
        self.clear_placements()
        self._PrototypesRel.ClearTargets()
        self._PrototypesRel.ClearConnections()

    def delete(self):
        self.clear_targets()
        super().delete()
