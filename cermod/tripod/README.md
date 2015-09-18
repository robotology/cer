# tripodMotionControl

How it works:

This device takes care of converting command from user to hw space and vice versa for encoder readings. It then take advantage of standard motionControl devices to send the converted command to HW, i.e. it uses a embObjMotionControl (or canBusMC) to send the values.

It can be instantiated as a device from robotInterface or as a standalone executable using yarpdev.
I'll put here the config files as documentation

ROBOT_INTERFACE version: <br>
icub_all.xml  (with only tripod device)

```
<robot name="iCubGenova04" build="1" portprefix="icub">
    <params file="hardware/electronics/pc104.xml" />
    <devices file="wrappers/motorControl/torso-tripod_wrapper.xml" />   <!--  New file -->
    <devices file="hardware/motorControl/torso-tripod.xml" />           <!--  New file -->
    <devices file="wrappers/motorControl/torso-mc_wrapper.xml" />
    <devices file="hardware/motorControl/torso-ems5-mc.xml" />
</robot>
```
torso-tripod.xml
```
<devices robot="iCubGenova04" build="1">
  <device name="tripod_mc" type="tripodMotionControl">
      <params file="general.xml" />

    <group name="GENERAL">
      <param name="Joints">    3                   </param>   <!--  to be refined, do we need it? -->
      <param name="AxisMap">   0 1 2               </param>   <!--  to be refined, do we need it? -->
      <param name="verbose">   true                </param>
    </group>

    <!-- This tells the tripod device to use the embObj 'torso_mc' device to send commands to HW after conversion is done -->
    <action phase="startup" level="5" type="attach">
      <paramlist name="networks">
          <elem name="FirstSetOfJoints">  torso_mc </elem>
      </paramlist>
    </action>

    <action phase="shutdown" level="5" type="detach" />

  </device>
</devices>
```

torso-tripod-wrapper.xml
```
<devices robot="iCubGenova04" build="1">
  <device name="torso_tripod_wrapper" type="controlboardwrapper2">
      <param name="threadrate">   10   </param>
      <paramlist name="networks">
	<!-- elem name hereafter are custom names that live only in this file, they are used in the attach phase -->
	  <elem name="FirstSetOfJoints">  0  2  0  2 </elem>
      </paramlist>

      <param name="period"> 20                  </param>
      <param name="name">   /icub/tripod        </param>
      <param name="ports">  torso               </param>
      <param name="joints"> 3                   </param>


      <action phase="startup" level="5" type="attach">
	  <paramlist name="networks">
	  <!-- The param value must match the device name in the corresponding emsX file -->
	      <elem name="FirstSetOfJoints">  tripod_mc </elem>

	  </paramlist>
      </action>

      <action phase="shutdown" level="5" type="detach" />
  </device>
</devices>
```

YARPDEV version: <br>
This was tested with Gazebo -- right now the tripod fake IK just add/subtract 5 degrees to the real values.

To run it:
 ``` yarpdev --from tripodConfig.ini ```

where tripodConfig contains the following

```
device controlboardwrapper2
subdevice tripodMotionControl
name /icubGazeboSim/tripod

[GENERAL]
Joints      3
AxisMap     0             1             2
verbose    false

[CONNECTION]
remote /icubGazeboSim/torso
local  /icubGazeboSim/torso_out
writeStrict on
```
