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

    redcube = Cube("/World/red")
    greencube = Cube("/World/green")
    bluecube = Cube("/World/blue") 
    instancer = Instancer("/World/mypointinstancer")

    instancer.add_target(redcube)
    instancer.add_target(greencube)
    instancer.add_target(bluecube)

    dim = 50
    dims=range(-dim,dim)
    instancer.protoindices = np.array([(x+y+z)%3 for x in dims for y in dims for z in dims])
    instancer.positions = np.array([[x,y,z] for x in dims for y in dims for z in dims])/dim

    redcube.color = (1.0,0.0,0.0)
    redcube.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    greencube.color = (0,1.0,0)
    greencube.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    bluecube.color = (0,0,1.0)
    bluecube.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    #end custom code
    isaac_sim_runner.run()
    simulation_app.close()

if __name__ == "__main__":
    main()
