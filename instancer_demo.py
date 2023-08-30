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

    # create cubes
    red,yellow,green,cyan,blue,purple = [Cube(F"/World/{color}") for color in {"red","yellow","green","cyan","blue","purple"}]

    # create instancer and add cubes as targets
    instancer = Instancer("/World/instancer")
    [instancer.add_target(cube) for cube in [red,yellow,green,cyan,blue,purple]]

    # designate transformations
    dim = 20
    dims=range(-dim,dim)
    instancer.protoindices = np.array([(x+y+z)%6 for x in dims for y in dims for z in dims])
    instancer.positions = np.array([[x,y,z] for x in dims for y in dims for z in dims])/dim
    instancer.rotations = np.array([np.array([x,y,z,0])/np.sqrt(x*x+y*y+z*z) for x in dims for y in dims for z in dims])
    instancer.scales = np.array([[50/(x%6+1),50/(y%6+1),50/(z%6+1)] for x in dims for y in dims for z in dims])/dim

    # targets will update their cloned instances when the prototype is updated
    red.color = (1.0,0.0,0.0)
    red.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    yellow.color = (1.0,1.0,0)
    yellow.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    green.color = (0,1.0,0)
    green.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))
    
    cyan.color = (0,1.0,1.0)
    cyan.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    blue.color = (0,0,1.0)
    blue.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    purple.color = (1.0,0,1.0)
    purple.scale = (1/(2*dim+1),1/(2*dim+1),1/(2*dim+1))

    #end custom code
    isaac_sim_runner.run()
    simulation_app.close()

if __name__ == "__main__":
    main()
