from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.simulation_context import SimulationContext

from assets import *
from isaac_stage.assets import AssetManager
from isaac_stage.terrain import *
from isaac_stage.stage_builder import ForestStageBuilder
from isaac_stage.appliers import apply_appliers, apply_color_to_prim, apply_default_ground_physics_material, apply_default_dirt_texture

"""
Source: https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/reference_python_snippets.html

According to the source above a valid raycast call requires the following: 
 1. the stage contains a physics scene, 
 2. all objects have collision meshes enabled 
 3. the play button has been clicked
"""

class IsaacSimRunner(object):
    """Runs omniverse."""
    def __init__(self):
        physics_dt = 1 / 100.0
        render_dt = 1 / 30.0
        self._world = SimulationContext(stage_units_in_meters=1.0, physics_dt=physics_dt, rendering_dt=render_dt, backend="torch")

    def on_physics_step(self,stepsize:float):
        pass 

    def run(self) -> None:
        """Step simulation based on rendering downtime"""

        self._world.reset()
        
        # change to sim running
        while simulation_app.is_running():
            #self.rayTraceRandomPoints()
            self._world.step(render=True)
        return

def main():    
    # open/load pre-built stage
    # get_context().open_stage(str(Path("PATH_TO_STAGE.usd*"),relative_pathing=True)
    # NOTE: This needs to happen before IsaacSimRunner()'s init
    # NOTE 2: Name collisions are a problem in general in this file.
    # This scene was saved with the same raytracing test that is initialized 
    # during the testing so those prims have name collisions.


    """Runs the simulation via the IsaacSimRunner."""
    isaac_sim_runner = IsaacSimRunner()
    simulation_app.update()

    # safe to set up here

    asset_manager = AssetManager()

    asset_manager.register("./sampletree.usdz", asset_scale=0.0375, applier=apply_default_ground_physics_material, area_factor=0.4)

    dim = 100 # both terrain and environment

    terrain = RoadsTerrain(terrain_unit=0.5, xdim=dim, ydim=dim, amp=0.2, spawn_radius=5.5,road_min_width=0.35, road_max_width=1.4, road_num=9, border_threshold=15.0
                                   ,applier=apply_appliers([apply_default_ground_physics_material, apply_default_dirt_texture]))

    environment = ForestStageBuilder(xdim=dim,ydim=dim,terrain=terrain,asset_manager=asset_manager)
    
    # build stage
    environment.build_stage(global_offset=[0,0,0],spawn_assets=True,asset_density=5)  

    isaac_sim_runner.run()

    # save stage
    print("Save Stage? [stages/demo_stage.usdc] (Y/n):")
    if input().lower().strip() != "n": 
        environment.save_stage("./stages/demo_stage.usdc")
        print("Stage Saved!")

    simulation_app.close()

if __name__ == "__main__":
    main()
