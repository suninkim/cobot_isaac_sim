from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"renderer": "RayTracedLighting", "headless": False})

import omni
from omni.isaac.core import World
from omni.isaac.core.prims.xform_prim import XFormPrim
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils import prims, rotations
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.types import ArticulationAction

# enable ROS2 bridge extension
enable_extension("omni.isaac.ros2_bridge")

simulation_app.update()

# Note that this is not the system level rclpy, but one compiled for omniverse
import argparse

import numpy as np
from isaac_ros2_messages.srv import EVDoorControl, Remove, Spawn

ELEVATOR_USD_PATH = "/extdisk/Projects/IsaacSim/Elevator/elevator_simulation_simple.usd"

class CobotEnv():
    def __init__(self, args):
        super().__init__("tutorial_subscriber")

        # setting up the world
        self.timeline = omni.timeline.get_timeline_interface()
        self.world = World(stage_units_in_meters=1.0)
        
        # setting up elevator controller
        add_reference_to_stage(usd_path=args.usd_path, prim_path='/World')
        self.cobot_controller = self.world.scene.add(Robot(prim_path='/World/Group/Cobot', name='cobot')).get_articulation_controller()
        self.world.scene.add(XFormPrim(prim_path='/World/Cobot', name='cobot'))
        
        self.world.reset()

        # setting up ROS communication
        self.spawned_list = []
        self.service_dict = {
            'ev_door_control': self.create_service(EVDoorControl, 'ev_door_control', self.srv_ev_door),
            'spawn': self.create_service(Spawn, 'spawn', self.srv_spawn),
            'remove': self.create_service(Remove, 'remove', self.srv_remove),
        }
        
    def srv_ev_door(self, req, res):
        door_position_l = req.door_position_l
        door_position_r = req.door_position_r
        self.ev_door_controller.apply_action(ArticulationAction(joint_positions=np.array([door_position_l,door_position_r])))

        return res

    def srv_spawn(self, req, res):
        names, classes, poses = req.names, req.classes, req.poses
        for i, pose in enumerate(poses):
            pos = pose.position
            quat = pose.orientation

            if classes[i] == 'ego':
                self.world.scene.get_object('around_ego_chassis').set_world_pose([pos.x, pos.y, pos.z], [quat.w, quat.x, quat.y, quat.z])              
            else:
                prims.create_prim(
                    f"/World/Obj_{i}",
                    position=np.array([pos.x, pos.y, pos.z]),
                    orientation=np.array([quat.w, quat.x, quat.y, quat.z]),
                    usd_path=names[i],
                )
                
            self.spawned_list.append(f"/World/Obj_{i}")

        return res

    def srv_remove(self, req, res):
        for prim_path in self.spawned_list:
            prims.delete_prim(
                prim_path
            )

        self.spawned_list = []
        return res

    def run_simulation(self):
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)            
            if self.world.is_playing():
                if self.world.current_time_step_index == 0:
                    self.world.reset()

        # Cleanup
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--usd-path', type=str, help='usd path for elevator simulation')
    args = parser.parse_args()

    subscriber = CobotEnv(args)
    subscriber.run_simulation()
