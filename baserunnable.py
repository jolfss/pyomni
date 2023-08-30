from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.simulation_context import SimulationContext

import numpy as np

#pyomni
from pyomni.cube import Cube
from pyomni.instancer import Instancer

class IsaacSimRunner(object):
    """Runs omniverse."""
    def __init__(self):
        physics_dt = 1 / 100.0
        render_dt = 1 / 30.0
        self._world = SimulationContext(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt, backend="torch")

    #NOTE: this particular callable is *not* optional if you want PhysX raytracing calls. [source?]
    def on_physics_step(self,stepsize:float):
        pass 

    def run(self) -> None:
        """Step simulation based on rendering downtime"""
        while simulation_app.is_running():
            self._world.step(render=True)
        return

def main():    
    """Runs the simulation via the IsaacSimRunner."""
    isaac_sim_runner = IsaacSimRunner()
    simulation_app.update()
    #begin custom code

    cube1 = Cube("/World/mycube")
    point_instancer1 = Instancer("/World/mypointinstancer")

    point_instancer1.add_target(cube1)
    point_instancer1.protoindices = np.array([0]*8)
    point_instancer1.positions = np.array([[x,y,z] for x in {-2,2} for y in {-2,2} for z in {-2,2}])

    #end custom code
    isaac_sim_runner.run()
    simulation_app.close()

if __name__ == "__main__":
    main()
