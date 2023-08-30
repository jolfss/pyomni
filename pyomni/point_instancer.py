# general python imports
from typing import List, Optional
import numpy as np

# omniverse imports
from pxr import UsdGeom, Gf, Vt, Usd, Sdf   

# library imports
from .core import *
from .prims import *

class PointInstancer(Prim):
    """Python wrapper class for the UsdGeom.PointInstancer."""
    def __init__(self, prim_path : str):
        super().__init__(prim_path)
        self._PointInstancer = UsdGeom.PointInstancer.Define(stage(),self.prim_path)
        self._PositionsAttr = self.instancer.GetPositionsAttr()
        self._ProtoIndicesAttr = self.instancer.GetProtoIndicesAttr()
        self._PrototypesRel = self.instancer.GetPrototypesRel()

        #Initialize
        self._PositionsAttr.Set(Vt.Vec3fArray(1))
        self._ProtoIndicesAttr.Set(Vt.IntArray(1))

    @property
    def instancer(self) -> UsdGeom.PointInstancer:
        return self._PointInstancer

    @property
    def positions(self):
        return self._PositionsAttr
    @positions.setter
    def positions(self, np_array : np.ndarray):
        self.positions.Set(Vt.Vec3fArray.FromNumpy(np_array))
    
    @property
    def protoindices(self):
        return self._ProtoIndicesAttr # TODO: These attribute references might expire..?
    @protoindices.setter
    def protoindices(self, new_protoindices : np.ndarray):
        min_protoindex = new_protoindices.min()
        max_protoindex = new_protoindices.max()
        if min_protoindex < 0:
            warn(F"PointInstancer recieved index that does not map to a target {min_protoindex}.")
        if max_protoindex >= len(self.targets):
            warn(F"PointInstancer recieved and index greater than any existing target {max_protoindex}.")
        self.protoindices.Set(Vt.IntArray.FromNumpy(new_protoindices))

    @property
    def prototype_relations(self):
        return self._PrototypesRel
    
    @property
    def targets(self) -> List[Sdf.Path]:
        return self.prototype_relations.GetTargets()
    def add_target(self, target : Prim):
        self.prototype_relations.AddTarget(target.prim_path)

    def remove_target(self, target : Prim):
        self.prototype_relations.RemoveTarget(target.prim_path)

    def clear_placements(self):
        self.protoindices.Clear()
        self.positions.Clear()
    
    def clear_targets(self):
        self.clear_placements()
        self.prototype_relations.ClearTargets()
        self.prototype_relations.ClearConnections()

    def delete(self):
        self.clear_targets()
        super().delete()
