# Reaching Usage

## Solver:

### Command-line parameters
- `--robot <robotName>`: name of the robot, specify the prefix name of the robot ports (default: `cer`).
- `--arm-type <left|right>`: which arm to use (default: `left`).
- `--get-bounds <on|off>`: specify if joints bound are retrieved by asking to the robot (default: `on`).
- `--verbosity <level>`: set the level of verbosity of the output of the solver (set level>=5 to enable derivative checker).

### RPC commands
- `get`: get the list of parameters currently set.
- `ask <parameters>`: asks the solver for a base position and joint configuration that enables reaching a specific targets:
  - on success returns [ack], followed by a list of robot configurations and a list of end effector poses found at the end of the optimization.
  - on failure returns [nack], followed by a list of robot configurations and a list of end effector poses found at the end of the optimization (no lists returned means something is wrong in the <parameters> field and the optimization could not start).

### RPC parameters

`<parameters>`=`(parameters (mode <mode>) (torso_heave <torso_heave>) (lower_arm_heave <lower_arm_heave>) (tol <tol>) (constr_tol <constr_tol>)) (q (<initial_joints>)) (target (<target>))`

- `(parameters ...)`:
  - `<mode>`=`"<target_type>+<constraints_type>+<finite_diff_type>"`: set the name of the mode used for the solver, in the form of a list of tokens:
    - `<target_type>`=`full_pose|pose_xyz`: set the type of targets:
      - `full_pose`: for position+orientation targets (dimension 7),
      - `xyz_pose`: for position targets (dimension 3).
    - `<constraints_type>`=`heave|no_heave|no_torso_heave|no_torso_no_heave|torso_yaw_no_heave`: specify a set of constraint to disable some DOF:
      - `heave`: all joints enabled,
      - `no_heave`: disable wrist heave,
      - `no_torso_heave`: disable torso,
      - `no_torso_no_heave`: disable torso and wrist heave
    - `<finite_diff_type>`=`forward_diff|central_diff`: set the type of finite difference used for jacobian computation:
      - `forward_diff`: forward finite difference,
      - `central_diff`: central finite difference.
  - `<torso_heave>`: set fixed value for torso heave (currently not implemented).
  - `<lower_arm_heave>`: set fixed value for wrist heave.
  - `<tol>`: set tolerance value for the solver.
  - `<constr_tol>`: set constraints tolerance value for the solver.
- `<initial_joints>`=`q1 q2 q3 ...`: initial value of the robot joints configuration to initialize the optimization process.
- `<target>`= target that the robot is asked to reach with respect to the reference coordinate system of the base link. 
  - `<target>`= `(x1 y1 z1 u1 u2 u3 t)`: cartesian coordinates of the target followed by orientation in axis/angle representation, in case of a position+orientation target.

## Controller

### Command-line parameters
- `--robot <robotName>`: name of the robot, specify the prefix name of the robot ports (default: `cer`).
- `--arm-type <left|right>`: which arm to use (default: `left`).
- `--verbosity <level>`: set the level of verbosity of the output (default: 0).
- `--stop-threshold-revolute <value>`: theshold for end of control for revolute joints (default: 2.0).
- `--stop_threshold_prismatic`: theshold for end of control for prismatic joints (default: `0.002`).
- `--T`: time constant for the joint control loop (default: `2.0`).
- `--Ts`: sample time for the joint control loop (default: `0.01`).

### RPC commands
- `set <T|Ts|verbosity|mode|torso_heave|lower_arm_heave|tol|constr_tol> <value>`: set the value of one of the parameters.
- `get <done|T|Ts|verbosity|mode|torso_heave|lower_arm_heave|tol|constr_tol>`: get the value of one of the parameters.
- `ask (<parameters>)`: ask the controller for a base position and joint configurations that enable reaching specific targets defined in the map frame.
- `go (<parameters>)`: ask the controller for a joint configurations that enable reaching specific targets defined in the base frame, then send the joint configuration to the joint controllers.
- `stop`: stop the current control.

### RPC parameters
- `<parameters>` mostly follows the same convention as the solver parameters: all the parameters are optional and will be filled by the module before forwarding the complete command to the solver module:
- `(q (<initial_joints>))`: if not specified, will be filled with the current state of the robot asking the joint controllers.

# Example
- Command sent to the controller module by the user:

`(parameters ((mode "full_pose+no_torso_no_heave+forward_diff") (torso_heave 0.0) (lower_arm_heave 0.02) (tol 0.0001) (constr_tol 1e-05))) (target (0.4 -0.4 0.7 0.7071068 0 0 0 0.7071068))`

- Completed command sent by the controller module to the solver module:

`ask (parameters ((mode "full_pose+no_torso_no_heave+forward_diff") (torso_heave 0.0) (lower_arm_heave 0.02) (tol 0.0001) (constr_tol 1e-05))) (target (0.4 -0.4 0.7 0.7071068 0 0 0 0.7071068)) (q (0.0 0.0 0.0 29.99 -21.23 16.91 45.45 93.98 73.29 0.0 0.022 0.026))`
