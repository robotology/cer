# mobile-reaching usage

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

`<parameters>`=`(parameters (mode <mode>) (torso_heave <torso_heave>) (lower_arm_heave <lower_arm_heave>) (tol <tol>) (constr_tol <constr_tol>)) (q (<initial_joints>)) (domain (<domain>)) (target (<target_list>))`

- `(parameters ...)`:
  - `<mode>`=`"<target_type>+<constraints_type>+<finite_diff_type>"`: set the name of the mode used for the solver, in the form of a list of tokens:
    - `<target_type>`=`full_pose|pose_xyz`: set the type of targets:
      - `full_pose`: for position+orientation targets (dimension 7),
      - `pose_xyz`: for position targets (dimension 3, currently not implemented).
    - `<constraints_type>`=`heave|no_heave|no_torso_heave|no_torso_no_heave|torso_yaw_no_heave`: specify a set of constraint to disable some DOF:
      - `heave`: all joints enabled (currently not implemented),
      - `no_heave`: disable wrist heave (currently not implemented),
      - `no_torso_heave`: disable torso (currently not implemented),
      - `no_torso_no_heave`: disable torso and wrist heave,
      - `torso_yaw_no_heave`: disable torso heave, keep torso yaw and disable wrist heave.
    - `<finite_diff_type>`=`forward_diff|central_diff`: set the type of finite difference used for jacobian computation:
      - `forward_diff`: forward finite difference,
      - `central_diff`: central finite difference.
  - `<torso_heave>`: set fixed value for torso heave (currently not implemented).
  - `<lower_arm_heave>`: set fixed value for wrist heave.
  - `<tol>`: set tolerance value for the solver.
  - `<constr_tol>`: set constraints tolerance value for the solver.
- `<initial_joints>`=`xb yb tb q1 q2 ...`: initial value of the base position (xb, yb), base orientation tb and robot joints configuration (q1, q2, ...) to initialize the optimization process.
- `<domain>`=`x1 y1 x2 y2 x3 y3 ...`: list of 2D points defining a polygon in which the robot base is constrained to stay (the base is free to move if the list contains less than 6 values, i.e. defining at least a triangle).
- `<target_list>`=`(<target1>) (<target2>) ...`: list of targets that the robot needs to be able to reach from a single base pose.
  - `<target1>`=`(x1 y1 z1)`: cartesian coordinates of the target, in case of position only target (currently not implemented),
  - `<target1>`=`(x1 y1 z1 u1 u2 u3 t)`: cartesian coordinates of the target followed by orientation in axis/angle representation, in case of a position+orientation target.

**Note: every position and orientations should be expressed with respect to the map frame.**

## Controller: 

### Command-line parameters

- `--robot <robotName>`: name of the robot, specify the prefix name of the robot ports (default: `cer`).
- `--arm-type <left|right>`: which arm to use (default: `left`).
- `--verbosity <level>`: set the level of verbosity of the output (default: 0).
- `--stop-threshold-revolute <value>`: theshold for end of control for revolute joints (default: 2.0).
- `--stop_threshold_prismatic`: theshold for end of control for prismatic joints (default: `0.002`).
- `--map-server`: port prefix of the map server (default: `/mapServer`).
- `--loc-server`: port prefix of the localization server (default: `/localizationServer`).
- `--nav-server`: port prefix of the navigation server (default: `/navigationServer`).
- `--map-name`: unused (default: `testMap`).
- `--T`: time constant for the joint control loop (default: `2.0`).
- `--Ts`: sample time for the joint control loop (default: `0.01`).

### RPC commands

- `set <T|Ts|verbosity|mode|torso_heave|lower_arm_heave|tol|constr_tol> <value>`: set the value of one of the parameters.
- `get <done|T|Ts|verbosity|mode|torso_heave|lower_arm_heave|tol|constr_tol>`: get the value of one of the parameters.
- `ask (<parameters>)`: ask the controller for a base position and joint configurations that enable reaching specific targets defined in the map frame.
- `askLocal (<parameters>)`: ask the controller for a base position and joint configurations that enable reaching specific targets defined in the local robot frame.
- `go (<parameters>)`: ask the controller for a base position and joint configurations that enable reaching specific targets defined in the map frame, then send the base position to the navigation module and the first joint configuration to the joint controllers. **(Note: navigation and joint motion are sent at the same time, without any check for potential collision, so this should probably be used for debugging purpose only)**.
- `stop`: stop the current control.

### RPC parameters

- `<parameters>` mostly follows the same convention as the solver parameters: only `(target (<target_list>)` is mandatory, all other parameters are optional and will be filled by the module before forwarding the complete command to the solver module:
  - `(q (<initial_joints>))`: if not specified, will be filled with the current state of the robot, first 3 DOF (base 2D position and orientation in the map) are filled by asking the localization server, all other DOF (joints) are filled by asking the joint controllers.
  - `(domain (<domain>))`: if not specified, will be filled by asking the map server the area in which the robot is currently (according to the localization server). *Note: In case no corresponding area is found in the map server, the solver may the area should be specified beforehand in the file `location.ini` loaded by the map server, or added during run-time before using this module. 

- Additional parameters:
  - `(<marginG|marginL> (<margins>))`: define a level of uncertainty on the specified targets by adding new targets with position and orientation shifted according to `<margins>`, with `marginG` the margins are defined along the axis of the global map frame, with `marginL` the margins are defined along the axis of the local target frame.
    - `<margins>`=`px py pz ox oy oz`: define the amount of motion in position (px, py, pz) and orientation (ox, oy, oz) along the respective axis of the selected frame (orientation margin is always centered on the target, i.e. no translation is resulting from the orientation margin).


# Example

- Command sent to the controller module by the user:

`askLocal ((parameters (mode "full_pose+torso_yaw_no_heave+central_diff") (tol 0.00000001)) (target ((0.4 -0.4 0.7 1 0 0 -1.57) (0.4 -0.4 0.8 1 0 0 -1.57))))`

- Completed command sent by the controller module to the solver module:

`ask (domain (8.3 1.3 10.9 0.8 10.9 1.15 10.5 1.15 10.5 2.5 11.1 2.5 11.8 4.6 8.75 5.4)) (parameters (mode "full_pose+torso_yaw_no_heave+central_diff") (tol 1e-08)) (q (9.24873475621598 2.04618971510724 -41.9575728809714 -0.0 -0.0 -0.0 29.5117745773781 54.0777681276328 49.2497841679977 -33.1309272550001 46.4695056341547 60.4673876758156 0.0189650443058725 0.0210013832268843 0.0190590759649363)) (target ((9.27875874769122 1.48130162005133 0.7 -0.878991992215933 0.337040067621978 -0.337308568580221 1.69862989847754) (9.27875874769122 1.48130162005133 0.8 -0.878991992215933 0.337040067621978 -0.337308568580221 1.69862989847754)))`


- Reply from the solver module:

`[ack] (q ((9.24856629445327 2.12205584580977 -56.8600707775936 0.0 0.0 0.0 10.9472319878922 34.0593319676408 43.5604244740798 -12.2045738770706 51.273761815799 45.2105006882131 0.0201645704368588 0.0191632769659527 0.0206714058851037) (9.24856629445327 2.12205584580977 -56.8600707775936 0.0 0.0 0.0 4.57522870904617 54.9999809741196 48.6249835886382 -32.2562672602733 47.2029992368698 59.5501641589647 0.0206115781609329 0.0208454029156618 0.0185360257769403))) (x ((9.2787585164361 1.48130163066557 0.699999767803497 -0.878991771622818 0.33704059446159 -0.337308617002823 1.69862973251971) (9.27875870186047 1.48130165134351 0.799999947448835 -0.878991911512994 0.337040201083821 -0.337308645528262 1.69862970840552)))`

- Reply from the controller module:

`[ack] (q ((-0.0508478624113735 0.0563044674544706 -14.9024978966222 0.0 0.0 0.0 10.9472319878922 34.0593319676408 43.5604244740798 -12.2045738770706 51.273761815799 45.2105006882131 0.0201645704368588 0.0191632769659527 0.0206714058851037) (-0.0508478624113735 0.0563044674544706 -14.9024978966222 0.0 0.0 0.0 4.57522870904617 54.9999809741196 48.6249835886382 -32.2562672602733 47.2029992368698 59.5501641589647 0.0206115781609329 0.0208454029156618 0.0185360257769403))) (x ((0.399999820932955 -0.400000146719404 0.699999767803497 -0.999999999999816 6.06434916667893e-07 7.95151344807255e-09 1.56999981729313) (0.399999944997096 -0.400000007371411 0.799999947448835 -0.999999999999987 1.63065088594301e-07 -1.22072541131853e-08 1.56999977439989)))`

 
