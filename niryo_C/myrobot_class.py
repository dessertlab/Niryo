from pyniryo import NiryoRobot, JointsPosition, PoseObject, ConveyorDirection
from pyniryo.api.enums_communication import ToolID
from typing import List, Tuple
from math import radians as _degToRad, degrees as _radToDeg
from threading import RLock
from math import pi

from enum import Enum

class Tools(Enum) :
    NONE = ToolID.NONE
    SMALL_GRIPPER = ToolID.GRIPPER_1
    LARGE_GRIPPER = ToolID.GRIPPER_2
    ADAPTIVE_GRIPPER = ToolID.GRIPPER_3
    VACUUM_PUMP_1 = ToolID.VACUUM_PUMP_1 
    VACUUM_PUMP_2 = ToolID.VACUUM_PUMP_2
    ELECTROMAGNET = ToolID.ELECTROMAGNET_1

class JointIndex(Enum):
    # Joint index constants
    BASE = 0
    SHOULDER = 1
    ELBOW = 2
    FOREARM_ROT = 3
    WRIST_ROT  = 4
    HAND = 5

class ToolState(Enum):
    # Tool State
    NO_STATE = 0
    GRIPPER_CLOSED = 1
    GRIPPER_OPEN = 2
    PUMP_PUSHED = 3
    PUMP_PULLED = 4
    MAGNET_INACTIVE = 5
    MAGNET_ACTIVE = 6

class MyRobot:
    # Poses dictionary Keys
    __JOINTS_KEY = "joints"
    __XYZ_POS_KEY = "xyz_pos"

    
    def __init__(self, ip: str, name:str = None, 
                 *, use_degrees:bool=True,
                    use_deprecated_methods:bool = True,
                   do_not_move:bool=False ):
        """
        Connect to the robot, calibrate, detect peripherals, and set up all the shit :).
        """
        self.robot = NiryoRobot(ip)
        self.robot.clear_collision_detected()

        # Personal Lock for threading
        self.lock = RLock()

        with self.lock:
            self.robot.calibrate_auto()

        # Detect tool
        self.tool_state = ToolState.NO_STATE
        self.tool_id = self.robot.get_current_tool_id()
        self.reset_tool_state()

        # Detect conveyors (only 1 in your case)
        conveyors = self.robot.get_connected_conveyors_id()
        # self.robot.set_conveyor()
        self.conveyor_id = conveyors[0] if conveyors else None

        # Detect Camera
        self.camera_ok = False
        try:
            img = self.robot.get_img_compressed()
            self.camera_ok = bool(img)
        except Exception:
            self.camera_ok = False

        # Set name and dicitonaries
        self.name = name if name else f"<{ip}>"
        self.saved_poses:dict = dict()
        self.saved_sequences:dict = dict()

        # Flags
        self._USE_DEG = use_degrees
        self._use_deprecated_methods = use_deprecated_methods
        self._do_not_move = do_not_move

        # Print summary
        self._print_status()

        with self.lock:
            self.robot.move_to_home_pose()

    # --------------------
    # Status
    # --------------------
    def _print_status(self):
        print(f"INFO::\tRobot: {self.name}")
        print(f"INFO::\t\tTool ID: {self.tool_id}")
        print(f"INFO::\t\tConveyor ID: {self.conveyor_id}")
        print(f"INFO::\t\tCamera: {'OK' if self.camera_ok else 'Not available'}")
        if len(self.saved_poses)>0:
            print(f"INFO::\t\t{len(self.saved_poses)} saved positions: {self.saved_poses.keys()}")
        if len(self.saved_sequences)>0:
            print(f"INFO::\t\t{len(self.saved_sequences)} saved positions: {self.saved_sequences.keys()}")
    
    
    ##############################################################################################
    # --------------------
    # General
    # --------------------
    @staticmethod
    def _angle_conversion(angle_input:float, 
                          *, convert_to_DEG:bool=False, input_is_DEG:bool) -> float:
        """
            converts any angle or angles to rad to make it niryo readable.
            The input is expected to be DEG if GLOBAL_use_deg is True, Rad otherwise
            if convertot o degree is true the logic it converts to degree
        """
        if convert_to_DEG:
            if input_is_DEG:                # no need for conversions
                return angle_input
            else:
                return _radToDeg(angle_input) 
            
        else:                       # Default: Convert to RAD
            if input_is_DEG:                # Deg -> Rad
                return _degToRad(angle_input)
            else:                   # No conversion needed
                return angle_input

    @staticmethod
    def _copy_pose(pose:JointsPosition|PoseObject) -> JointsPosition|PoseObject:
        if isinstance(pose, JointsPosition):
            l = pose.to_list()
            return JointsPosition(*l)
                
        elif isinstance(pose, PoseObject):
            l = pose.to_list()
            return PoseObject(*l)

        else:
            print(f"ERROR::\tCan't copy pose of type {type(pose)}. Use either JointsPosition or PoseObject")

    def wait(self, seconds:float):
        if seconds>0:
            self.robot.wait(seconds)

    # --------------------
    # Movement (Joints)
    # --------------------
    def move_joints(self, joints:JointsPosition|List[float]):
        """Move to absolute joint positions (list or JointsPosition)."""
        if isinstance(joints, list):
            for i in range(len(joints)):
                joints[i] = MyRobot._angle_conversion(joints[i], input_is_DEG=self._USE_DEG)
            joints = JointsPosition(*joints)

        if not self._do_not_move:
            if self._use_deprecated_methods:
                with self.lock:
                    self.robot.move_joints(joints)
            else:
                self.robot.move(joints)

    def get_current_joints(self) -> JointsPosition:
        with self.lock:
            return self.robot.get_joints()

    def compute_shifted_joints(self, joints:JointsPosition, shifts:List[Tuple[int, float]]|Tuple[int, float]) -> JointsPosition:
        if not joints:
            joints = self.get_current_joints()
        
        if not isinstance(shifts, list):
            shifts = [shifts]
        for shift in shifts:
            joints = self.compute_single_shift_joint(joints, shift)
        return joints

    def compute_single_shift_joint(self, joints:JointsPosition, shift:Tuple[int, float]) -> JointsPosition:
        if not joints:
            joints = self.get_current_joints()
        
        idx, delta = shift
        joints[idx] += MyRobot._angle_conversion(delta, input_is_DEG=self._USE_DEG)
        return joints

    def shift_joint(self, j_index: int, delta: float) -> JointsPosition:
        """Shift a single joint by delta (radians)."""
        if j_index < 0 or j_index > 5:
            print(f"ERROR::\t{self.name}\tInvalid j_index {j_index}, should be between 0 and 5")
            return None
        
        joints = self.get_current_joints()
        joints = self.compute_single_shift_joint(joints, shift=(j_index, delta))
        self.move_joints(joints)
        return joints

    # wrappers for each joint
    def shift_joint_base(self, delta:float) -> JointsPosition:      return self.shift_joint(JointIndex.BASE, delta)
    def shift_joint_shoulder(self, delta:float) -> JointsPosition:  return self.shift_joint(JointIndex.SHOULDER, delta)
    def shift_joint_elbow(self, delta:float) -> JointsPosition:     return self.shift_joint(JointIndex.ELBOW, delta)
    def shift_joint_forearm(self, delta:float) -> JointsPosition:   return self.shift_joint(JointIndex.FOREARM_ROT, delta)
    def shift_joint_wrist(self, delta:float) -> JointsPosition:     return self.shift_joint(JointIndex.WRIST_ROT, delta)
    def shift_joint_hand(self, delta:float) -> JointsPosition:      return self.shift_joint(JointIndex.HAND, delta)

    # --------------------
    # Movement (XYZ Position)
    # --------------------
    def move_to_xyz_position(self, xyz_pos: PoseObject|list):
        """Move to absolute pose (list or PoseObject)."""
        if isinstance(xyz_pos, list):
            xyz_pos = PoseObject(*xyz_pos)

        if not self._do_not_move:
            if self._use_deprecated_methods:
                with self.lock:
                    self.robot.move_pose(xyz_pos)
            else:
                with self.lock:
                    self.robot.move(xyz_pos)

    def get_current_xyz_position(self) -> PoseObject:
        """Return current pose as PoseObject."""
        with self.lock:
            return self.robot.get_pose()

    # --------------------
    # Movement (other)
    # --------------------
    def move(self, target:JointsPosition|PoseObject) :
        if isinstance(target, JointsPosition):
            self.move_joints(target)
        elif isinstance(target, PoseObject):
            self.move_to_xyz_position(target)
        else:
            print(f"ERROR::\t<{self.name}>\t Unrecognized type '{type(target)}'. Please use one of {(JointsPosition, PoseObject)}")

    @staticmethod
    def interpolate_joints_by_delta(start_jts:JointsPosition, target_jts:JointsPosition, max_delta=[pi/4., pi/3., pi/3., pi/2., pi/2., pi/2.],
                                    *, name:str = None):
        import numpy as np
        
        if max_delta is None or len(max_delta)==0:
            return [target_jts]
        else:
            max_delta = np.array(max_delta, dtype=float)
        
        start = np.array(start_jts.to_list(), dtype=float)
        target = np.array(target_jts.to_list(), dtype=float)
        
        # Lunghezza delle rotazioni lungo ogni giunto
        diffs = np.abs(target - start)

        # required steps per joint
        steps = np.ceil( diffs / max_delta)

        # numero di step basato sulla differenza massima ponderata
        n_steps = int(np.max(steps))

        alphas = np.linspace(0., 1., n_steps + 1)
        poses = [(1. - a) * start + a * target for a in alphas]
        cause = np.argmax(steps)
        print(f"INFO::\t{f'{name}:\t' if name else ''}Movement broken in {len(poses)-1} submovement{f"s (cause: {JointIndex(cause)}  {_radToDeg(np.max(diffs)):.2f}° where max step is {_radToDeg(max_delta[cause]):.2f}°)" if len(poses)>2 else ""}")
        return [JointsPosition(*(p.tolist())) for p in poses[1:-1]]+[target_jts]



    # --------------------
    # Kinematics and conversions
    # --------------------
    def joints_to_pose(self, joints: JointsPosition=None) -> PoseObject:
        if not joints:
            joints = self.get_current_joints()
        return self.robot.forward_kinematics(joints)

    def xyzPosition_to_joints(self, xyz_position: PoseObject=None) -> JointsPosition:
        if not xyz_position:
            xyz_position = self.get_current_xyz_position()
        return self.robot.inverse_kinematics(xyz_position)

    # --------------------
    # Saved positions
    # --------------------
    def home(self) -> Tuple[JointsPosition, PoseObject]:
        with self.lock:
            self.robot.move_to_home_pose()
            jts, xyz = self.robot.get_joints(), self.robot.get_pose()
        return jts, xyz
    
    def save_pose(self, name:str, pose:PoseObject|JointsPosition|List[float] = None, ptype:str=None,
                  *, overwrite:bool=False) -> PoseObject|JointsPosition:
        """
        Save a joints position or Pose object in the saved poses dictionary.
        - If `pose` is None/empty, uses current robot joints/pose.
        - If `pose` is a list of 6 floats, convert it.

        Always uses the type specified in ptype.
        - If 'ptype' is None or empty, it tries to deduce it from pose
        """
        # Check for errors
        if ptype not in (MyRobot.__XYZ_POS_KEY, MyRobot.__JOINTS_KEY):
            if isinstance(pose, JointsPosition):
                ptype = MyRobot.__JOINTS_KEY
            elif isinstance(pose, PoseObject):
                ptype = MyRobot.__XYZ_POS_KEY
            else:
                print(f"ERROR::\t<{self.name}>\t unknown pose-type '{ptype}'. Please use one of {(MyRobot.__XYZ_POS_KEY, MyRobot.__JOINTS_KEY)}")
                return None
        
        if not pose:
            match ptype:
                case MyRobot.__JOINTS_KEY:
                    pose:JointsPosition = self.get_current_joints()
                case MyRobot.__XYZ_POS_KEY:
                    pose:PoseObject = self.get_current_xyz_position()
        elif isinstance(pose, list) and len(pose)==6:
            match ptype:
                case MyRobot.__JOINTS_KEY:
                    pose:JointsPosition = JointsPosition(*pose)
                case MyRobot.__XYZ_POS_KEY:
                    pose:PoseObject = PoseObject(*pose)
        elif isinstance(pose, dict) and len(pose)==6:
            match ptype:
                case MyRobot.__JOINTS_KEY:
                    print(f"ERROR::\t<{self.name}>\t JointsPosition poses should not be dictionaries! Use lists instead")
                    return None
                case MyRobot.__XYZ_POS_KEY:
                    pose:PoseObject = PoseObject(**pose)
        elif not isinstance(pose, JointsPosition) and not isinstance(pose, PoseObject):
            print(f"ERROR::\t<{self.name}>\t Pose should be either a JointsPosition, PoseObject or a list of 6 floats.\n\t\tGot {type(pose)}, {len(pose)}")
            return None
        
        if (ptype==MyRobot.__JOINTS_KEY and not isinstance(pose, JointsPosition)) or (ptype==MyRobot.__XYZ_POS_KEY and not isinstance(pose, PoseObject)):
            print(f"ERROR::\t<{self.name}>\t Pose-type is '{ptype}' but provided pose is {(type(pose))}. What do you expect me to do?")
            return


        # Safe save
        if name in self.saved_poses:
            print(f"WARNING::\t<{self.name}>\t Position by the name '{name}' already in saved positions. {'Overwriting' if overwrite else 'Not overwriting.. saving aborted'}")
            if not overwrite: return None
        pose_copy = MyRobot._copy_pose(pose)
        self.saved_poses[name] = (ptype, pose_copy)
        return pose

    def save_current_pose(self, name:str, as_joint:bool=True) -> JointsPosition|PoseObject:
        if as_joint:
            return self.save_pose(name, pose=self.get_current_joints(), ptype=MyRobot.__JOINTS_KEY)
        else:
            return self.save_pose(name, pose=self.get_current_xyz_position(), ptype=MyRobot.__XYZ_POS_KEY)
        
    def get_saved_pose(self, name:str) -> Tuple[str, JointsPosition|PoseObject]:
        if name in self.saved_poses:
            pos_type, data = self.saved_poses[name]
            return pos_type, MyRobot._copy_pose(data)
        else:
            print(f"ERROR::\t<{self.name}>\t No saved pose by the name '{name}'")

    def run_pose(self, name: str, interp_delta:List[float]=None, intra_interp_delay:float=0.):
        """
        Move to a saved joints position or pose.
        """
        if name not in self.saved_poses:
            print(f"ERROR::\t{self.name}\tERROR: Unkown saved position name: '{name}'!")
            return

        pos_type, data = self.saved_poses[name]
        if pos_type == MyRobot.__JOINTS_KEY:
            interp_jts  = MyRobot.interpolate_joints_by_delta(self.get_current_joints(), data, interp_delta)
            for jts in interp_jts:
                
                if intra_interp_delay>0:    self.wait(intra_interp_delay)

                self.move_joints(jts)
        elif pos_type == MyRobot.__XYZ_POS_KEY:
            self.move_to_xyz_position(data)

    # --------------------
    # Saved sequences
    # --------------------
    def save_sequence(self, name: str, positions: list, overwrite: bool=False):
        if name in self.saved_sequences:
            print(f"WARNING::\t<{self.name}>\t Sequence by the name '{name}' already in saved sequences. {'Overwriting' if overwrite else 'not overwriting.. saving aborted'}")
            if not overwrite: return

        # normalize elements: either "pose_name" or {"wait": seconds}
        normalized = []
        for step in positions:
            if isinstance(step, str):
                normalized.append(step)             # keep pose-name strings for compatibility
            elif isinstance(step, dict):
                if "wait" in step:
                    normalized.append({"wait": float(step["wait"])})
                elif "gripper" in step:
                    normalized.append({"gripper": step["gripper"]})
                elif "pump" in step:
                    normalized.append({"pump": step["pump"]})
                elif "magnet" in step:
                    normalized.append({"magnet": step["magnet"]})
                elif "conveyor" in step:
                    normalized.append({"conveyor": step["conveyor"]})
                elif "tool" in step:
                    normalized.append({"tool": step["tool"]})
                else:
                    print(f"ERROR::\t<{self.name}>\t Invalid sequence step: {step}")
            else:
                print(f"ERROR::\t<{self.name}>\t Invalid sequence step: {step}")
        self.saved_sequences[name] = normalized
        # print(f"INFO::\t{self.name}\tSaved sequence '{name}': {normalized}")

    def get_saved_sequence(self, seq_name:str) -> List[str]:
        if seq_name not in self.saved_sequences:
            print(f"ERROR::\t<{self.name}>\t Unknown sequence by the name '{seq_name}'.")
            return []
        return self.saved_sequences[seq_name][:]

    def run_sequence(self, sequence: List[str] | str,
                        *,
                        interp_delta:List[float]=None,
                        verbose:bool=False,
                        inter_poses_delay:float = 0.,
                        intra_interp_delay:float = 0.005,
                        reverse:bool = False,
                        ):
        if isinstance(sequence, str):
            if sequence in self.saved_sequences:
                if verbose:     print("Running sequence", sequence)
                s_name = sequence
                sequence = self.saved_sequences[sequence]
            else:
                print(f"ERROR::\t{self.name}:\t Unkown saved sequence name: '{sequence}'!")
                return
        else:
            s_name = sequence

        for step in sequence[::(1 if not reverse else -1)]:
            # step may be a pose name string:
            if isinstance(step, str):
                if verbose:     print(f"ERROR::\t{self.name}:\tRunning pose '{step}' of sequence '{s_name}'")
                self.run_pose(step, interp_delta=interp_delta, intra_interp_delay=intra_interp_delay)
            # or a dict like {"wait": seconds}
            elif isinstance(step, dict):
                if "wait" in step:
                    secs = float(step["wait"])
                    # print(f"Waiting for {secs} s")
                    self.wait(secs)
                elif "tool" in step:
                    match step["tool"]:
                        case "grasp":
                            self.tool_grasp()
                        case "release":
                            self.tool_release()
                elif "gripper" in step:
                    match step["gripper"]:
                        case "open":
                            self.gripper_open()
                        case "grasp":
                            self.tool_grasp()
                        case "release":
                            self.tool_release()
                        case "close":
                            self.gripper_close()
                elif "pump" in step:
                    match step["pump"]:
                        case "pull":
                            self.pump_pull()
                        case "grasp":
                            self.tool_grasp()
                        case "release":
                            self.tool_release()
                        case "push":
                            self.pump_push()
                elif "magnet" in step or "electromagnet" in step:
                    match step["pump"]:
                        case "activate":
                            self.electromagnet_activate()
                        case "grasp":
                            self.tool_grasp()
                        case "release":
                            self.tool_release()
                        case "deactivate":
                            self.electromagnet_deactivate()
                elif "conveyor" in step:
                    conv_op:dict = step["conveyor"]
                    op = conv_op.get("command", "run").lower()
                    if op=="run":
                        dir = conv_op.get("direction", ConveyorDirection.FORWARD)
                        speed = conv_op.get("speed", 100)
                        self.run_conveyor(speed=speed, direction=dir)
                    elif op=="stop":
                        self.stop_conveyor()

                else: 
                    print(f"ERROR::\t{self.name}\tInvalid sequence step: {step}")
            else:
                print(f"ERROR::\t{self.name}\tInvalid sequence step: {step}")
            
            # Inter poses delay
            if inter_poses_delay>0: self.wait(inter_poses_delay)
    
    def load_from_json(self, path:str, 
                       *, overwrite:bool=False):
        """
            Loads poses and sequences from a Json file
            - If 'overwrite' is True, it will overwrite any pose or sequence name already present
            - If 'is_input_degree' is True, it will convert anyjoint and pitch/yaw/roll to rad
            - - If 'is_input_degree' is None (default) it will use the private field _USE_DEG
        """
        import json 
        with open(path, "r") as f:
            data = json.load(f)

        for pose_name, pose_data in data.get("poses", {}).items():
            ptype = pose_data["type"]
            punit = pose_data.get("unit", "rad").lower()

            if punit not in ("deg", "degree", "rad", "radians"):
                print(f"ERROR::\t<{self.name}>\t pose unit '{punit}' unknown. Use one of {("deg", "degree", "rad", "radians")}")
            elif punit == "degree":
                    punit = "deg"

            if ptype in (MyRobot.__JOINTS_KEY, MyRobot.__XYZ_POS_KEY):
                pose = pose_data["values"]
                if punit == "deg":
                    match ptype:
                        case MyRobot.__JOINTS_KEY:
                            for idx in range(len(pose)):
                                pose[idx] = _degToRad(pose[idx])
                        case MyRobot.__XYZ_POS_KEY:
                            for _k in ("pitch", "yaw", "roll"):
                                pose[_k] = _degToRad(pose[_k])
                
                self.save_pose(pose_name, pose, ptype, overwrite=overwrite)
            else:
                print(f"ERROR::\t<{self.name}>\t pose type '{ptype}' unknown. Use one of {(MyRobot.__JOINTS_KEY, MyRobot.__XYZ_POS_KEY)}")
                return

        for seq_name, seq_data in data.get("sequences", {}).items():
            for step in seq_data:
                if isinstance(step, str) and step not in data.get("poses", {}):
                    print(f"WARNING::\t<{self.name}>\t Sequence '{seq_name}' references unknown pose '{step}'")
            self.save_sequence(seq_name, seq_data, overwrite=overwrite)

    # --------------------
    # Tools
    # --------------------
    def tool_grasp(self):
        if self.tool_id != ToolID.NONE:
            with self.lock:
                self.robot.grasp_with_tool()
            self.update_tool_state(False)

    def tool_release(self):
        if self.tool_id != ToolID.NONE:
            with self.lock:
                self.robot.release_with_tool()
            self.update_tool_state(True)
        
        # tool specific
    
    def gripper_open(self):
        with self.lock:
            if self.tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3):
                self.robot.open_gripper()
                self.update_tool_state(True)
                return
        print(f"WARNING::\t{self.name}\tNo gripper attached. Found: {Tools(self.tool_id)}")
    def gripper_close(self):
        with self.lock:
            if self.tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3):
                self.robot.close_gripper()
                self.update_tool_state(False)
                return
        print(f"WARNING::\t{self.name}\tNo gripper attached. Found: {Tools(self.tool_id)}")
    def pump_pull(self):
        with self.lock:
            if self.tool_id in (ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2):
                self.robot.pull_air_vacuum_pump()
                self.update_tool_state(False)
                return
        print(f"WARNING::\t{self.name}\tNo vacuum pump attached. Found: {Tools(self.tool_id)}")
    def pump_push(self):
        with self.lock:
            if self.tool_id in (ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2):
                self.robot.push_air_vacuum_pump()
                self.update_tool_state(True)
                return
        print(f"WARNING::\t{self.name}\tNo vacuum pump attached. Found: {Tools(self.tool_id)}")
    def electromagnet_activate(self):
        with self.lock:
            if self.tool_id in (ToolID.ELECTROMAGNET_1, ):
                self.robot.activate_electromagnet()
                self.update_tool_state(False)
                return
        print(f"WARNING::\t{self.name}\tNo electromagnet attached. Found: {Tools(self.tool_id)}")
    def electromagnet_deactivate(self):
        with self.lock:
            if self.tool_id in (ToolID.ELECTROMAGNET_1, ):
                self.robot.deactivate_electromagnet()
                self.update_tool_state(True)
                return
        print(f"WARNING::\t{self.name}\tNo electromagnet attached. Found: {Tools(self.tool_id)}")
    
    #    # Tool State 
    def reset_tool_state(self):
        # with self.lock:
        if self.tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3):
            self.tool_state = ToolState.GRIPPER_CLOSED
        elif self.tool_id in (ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2):
            self.tool_state = ToolState.PUMP_PUSHED
        elif self.tool_id in (ToolID.ELECTROMAGNET_1, ):
            self.tool_state = ToolState.MAGNET_INACTIVE 
        else:
            self.tool_state = ToolState.NO_STATE
        
    def update_tool_state(self, isReleased:bool):
        # with self.lock:
        if self.tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3):
            self.tool_state = ToolState.GRIPPER_OPEN if isReleased else ToolState.GRIPPER_CLOSED 

        elif self.tool_id in (ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2):
            self.tool_state = ToolState.PUMP_PUSHED  if isReleased else ToolState.PUMP_PULLED

        elif self.tool_id in (ToolID.ELECTROMAGNET_1, ):
            self.tool_state = ToolState.MAGNET_INACTIVE if isReleased else ToolState.MAGNET_ACTIVE

        else:
            self.tool_state = ToolState.NO_STATE
                
    def get_tool_state(self):
        return self.tool_state
    
    def is_tool_gripper(self) -> bool:
        return self.tool_id in (ToolID.GRIPPER_1, ToolID.GRIPPER_2, ToolID.GRIPPER_3)
    
    def is_tool_vacuum_pump(self) -> bool:
        return self.tool_id in (ToolID.VACUUM_PUMP_1, ToolID.VACUUM_PUMP_2)
    
    def is_tool_electromagnet(self) -> bool:
        return self.tool_id in (ToolID.ELECTROMAGNET_1, )
    
    def is_tool_released(self) -> bool:
        return self.tool_state in (ToolState.GRIPPER_OPEN, ToolState.PUMP_PUSHED, ToolState.MAGNET_INACTIVE)

    def is_tool_grasping(self) -> bool:
        return self.tool_state in (ToolState.GRIPPER_CLOSED, ToolState.PUMP_PULLED, ToolState.MAGNET_ACTIVE)

    # --------------------
    # Conveyor
    # --------------------
    CONVEYOR_FORWARD = ConveyorDirection.FORWARD
    CONVEYOR_BACKWARD = ConveyorDirection.BACKWARD
    def run_conveyor(self, speed=50, direction=CONVEYOR_FORWARD):
        """Run conveyor if connected (direction: 1=fwd, -1=back)."""
        if not self.conveyor_id:
            print(f"ERROR::\t{self.name}\tNo conveyor detected")
            return

        if not isinstance(direction, ConveyorDirection):
            if direction>0:
                direction = MyRobot.CONVEYOR_FORWARD
            elif direction<0:
                direction = MyRobot.CONVEYOR_BACKWARD
            else:
                self.stop_conveyor()
                return
        
        if speed<0: speed = 0
        elif speed>100: speed = 100 

        with self.lock:
            self.robot.run_conveyor(self.conveyor_id, speed, direction)

    def stop_conveyor(self):
        """Stop conveyor if connected."""
        if not self.conveyor_id:
            print(f"ERROR::\t{self.name}\tNo conveyor detected")
            return
        with self.lock:
            self.robot.stop_conveyor(self.conveyor_id)

    # --------------------
    # Camera
    # --------------------
    def snapshot(self, filename="snapshot.jpg"):
        """Save a camera snapshot to a file."""
        if not self.camera_ok:
            print(f"ERROR::\t{self.name}\tCamera not available")
        with self.lock:
            img = self.robot.get_img_compressed()
        with open(filename, "wb") as f:
            f.write(img)


    # --------------------
    # Cleanup
    # --------------------
    def disconnect(self):
        with self.lock:
            if not self._do_not_move:
                self.robot.move_to_home_pose()
            print(f"{self.name}: Disconnecting")
            self.robot.close_connection()

