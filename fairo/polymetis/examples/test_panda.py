
from polymetis import GripperInterface, RobotInterface
import torch


# robot = RobotInterface(
#     ip_address="10.16.254.173", enforce_version=False, port=50051
# )
# robot = RobotInterface(
#     ip_address="10.105.148.13", enforce_version=False, port=50051
# )
gripper = GripperInterface(ip_address="10.16.254.173", port=50052)


state = gripper.get_state()
print(f"current gripper position: {state.width}")

state_log = gripper.goto(width=0.08, speed=0.03, force=0.05)


# # Get updated joint positions
# joint_positions = robot.get_joint_positions()
# print(f"New joint positions: {joint_positions}")


# from polymetis import GripperInterface, RobotInterface
# import torch
# import time


# def main(ctrl_type="impedance"):
#     robot = RobotInterface(
#        ip_address="localhost", enforce_version=False, port=50051
#     )

#     gripper = GripperInterface(ip_address="localhost", port=50052)
#     state = gripper.get_state()
#     print(f"current gripper position: {state.width}")


#     state_log = gripper.goto(width=0.01, speed=0.01, force=0.05)


#     # Reset
#     robot.go_home()
#     print("***HHIHIHIHI")


#     # Get joint positions
#     joint_positions = robot.get_joint_positions()
#     ee_pos, ee_quat = robot.get_ee_pose()
#     print(f"Current joint positions: {joint_positions}")


#     robot_state = robot.get_robot_state()
#     ee_pose = robot.get_ee_pose()
#     # robot_state.ee_pose[:] = torch.cat(ee_pose).tolist()


#     print("type of robot state",type(robot_state))
#     print("robot_state.ee_pose",robot_state)
#     # get_ee_pose_mat()




#     from polymetis_pb2 import LogInterval, RobotState, ControllerChunk, Empty
#     import torchcontrol as toco
#     metadata = robot.grpc_connection.GetRobotClientMetadata(Empty())


#     if ctrl_type == "impedance":
#         ctrl = robot.start_joint_impedance()
#         increment = 0.05
#     elif ctrl_type == "osc":
#         from furniture_bench.controllers.osc import osc_factory
#         from furniture_bench.config import config
#         ee_pos_current, ee_quat_current = robot.get_ee_pose()
#         ee_pos_current = torch.tensor(ee_pos_current, dtype=torch.float32)
#         ee_quat_current = torch.tensor(ee_quat_current, dtype=torch.float32)
#         reset_joints = torch.tensor(config["robot"]["reset_joints"])
#         kp = torch.tensor([80, 80, 80, 50.0, 40.0, 50.0])
#         kv = torch.ones((6,)) * torch.sqrt(kp) * 2.0
#         ctrl = osc_factory(
#             ee_pos_current=ee_pos_current,
#             ee_quat_current=ee_quat_current,
#             init_joints=reset_joints,
#             kp=kp,
#             kv=kv,
#             position_limits=torch.tensor(config["robot"]["position_limits"]),
#         )
#         robot.send_torch_policy(torch_policy=ctrl, blocking=False)
#         increment = 0.02



#     for c in range(100):
#         print(c)
#         increment = -increment
#         for i in range(20):
#             state = robot.get_robot_state()
#             # print(state)
#             if ctrl_type == "impedance":
#                 joint_positions += torch.Tensor([0.0, 0.0, increment*0.2, 0.0, 0.0, increment, 0.0])
#                 robot.update_desired_joint_positions(joint_positions)
#             elif ctrl_type == "osc":
#                 ee_pos += torch.Tensor([0.0, 0.0, increment])
#                 robot.update_desired_ee_pose(position=ee_pos, orientation=ee_quat)


#             time.sleep(0.1)




# if __name__ == "__main__":
#    ctrl_type = "impedance"
#    main(ctrl_type)
