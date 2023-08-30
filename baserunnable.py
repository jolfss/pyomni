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
    cube2 = Cube("/World/mycube2")
    instancer = Instancer("/World/mypointinstancer")

    instancer.add_target(cube1)
    instancer.add_target(cube2)

    dim = 50
    dims=range(-dim,dim)
    instancer.protoindices = np.array([(x+y+z)%2 for x in dims for y in dims for z in dims])
    instancer.positions = np.array([[x,y,z] for x in dims for y in dims for z in dims])/dim

    cube1.color = (1.0,0.0,0.0)
    cube1.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    cube2.color = (0.25,0.75,1.0)
    cube2.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    #end custom code
    isaac_sim_runner.run()
    simulation_app.close()

if __name__ == "__main__":
    main()
