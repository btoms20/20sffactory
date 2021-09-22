# RepRapFirmware Changes and Source Code Compilation Info

## Kinematics
#### Various Configurations
- [x] **Arm with swivel base**
   - The standard set up
   - Results in a swept work area (print bed) (180 degree donut style reachable area)
- [ ] **Arm with linear base** 
   - Might be the easiest kinematic implementation (arm controls Y & Z, X is just a simple cartesian linear translation)
   - Results in a rectangular work area (print bed)
- [ ] **Arm with linear and swivel base** 
   - Multiple solutions for most reachable points
   - Could lock-out swivel for faster kinematic calculations
   - The [ScaraKinematics.ccp](https://github.com/Duet3D/RepRapFirmware/blob/master/src/Movement/Kinematics/ScaraKinematics.cpp) takes the multiple solutions into account (we can do the same)


## Help Compiling RRF Source
#### Compile Source v3.3
- Basically [follow this guide](https://github.com/Duet3D/RepRapFirmware/wiki/Building-RepRapFirmware)
- But instead of cloning the branches for each library. Download the zip of the source files under the 3.3 tag
- You also need CANlib
- Build Targets (build them in this order)
   - CANlib - SAME5x
   - CoreN2G - SAME5x_CAN_SDHC_USB_RTOS
   - FreeRTOS - SAME51
   - RRFLibraries - SAME51_RTOS
   - RepRapFirmware - Duet3Mini5plus
- You also need to install the [crc32appender](https://github.com/Duet3D/RepRapFirmware/pull/322/commits/b188210237a7787fd9083fa19d156484741b339d) bin in the Duet repo
   - `chmod +x` it
   - remove it from quarantine (if on mac osx) (`sudo xattr -r -d com.apple.quarantine <file name>`)
   - copy into `/usr/local/bin` so eclipse can pick it up during the build

#### Adding custom kinematics
- Basically [follow this guide](https://duet3d.dozuki.com/Wiki/Configuring_RepRapFirmware_for_a_FiveAxisRobot) using the files in the RRF directory

#### Uploading Firmware
- Building the source files (with our custom kinematics) will result in a `Deut3_Mini5plus.uf2` file
- Using DWC web interface, Upload system files and select the `.uf2` file
- DWC will check it and ask if you'd like to restart & install the new firmware

#### Configure DWC
- Follow the README in the DWC directory for help on configuring DWC to use the new kinematic profile

## Resources
- [GCODE Dictionary](https://duet3d.dozuki.com/Wiki/Gcode#Section_G92_Set_Position)
- [RepRapFirmware Forum](https://duet3d.dozuki.com/c/RepRapFirmware)
