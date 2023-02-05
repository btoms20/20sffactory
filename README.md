# 20sffactory Robot Arm
Iterating on the 20sffactory robot arm 🦾🤖

Hi! 👋
 
This repo is home to all the notes, thoughts and future plans I have for my build of the awesome [20sf Robot Arm](https://www.20sffactory.com/robot/about). Below, you'll find some basic info covering some of the physical and electrical attributes of the arm and the software stack used to control it; There's also a TODO list of features I'm hoping to add in the near future.  

If you have any questions or comments, feel free to post on the [discussions board](https://github.com/btoms20/20sffactory/discussions)! 🙋
   

## 🦾 Structure
- 3 axis palletizer robot arm, with an additional axis via the linear rails
- The frame is 3D Printed (all parts can be printed on a prusa mini (print volume - 180mm^3)
- Hardware is easily sourced online

   
## 🧠 Logics   
- Duet 3 mini 5+ running RepRapFirmware with custom IK code    
- DWC interface with http gcode server exposed
- Google Coral Dev Board, responsible for...
   - Processes Camera input
   - Runs OpenCV for basic edge detection
   - Passes images off to a remote ML model for advanced object detection
   - Interfaces with the duet web server via http requests
  

## 🔌 Electronics & Hardware
- 24v 10amp meanwell power supply
- 2 High Torque Nema 17 Steppers running at 24v (one for each arm)
- 2 Regular Nema 17 Steppers running at 24v (one for the rotational axis and one for the linear rails)
- 1 tiny stepper motor for gripper (with bi-polar conversion)
- 1 5v Servo for the camera tilt mount
- No end stops (all homing is done via stall detection on the Duet)

 
## ✅ TODOs   
- [x] Build robot
- [x] Design and print a case for the Mini 5+ 
- [x] Electronics (cable routing, wiring, soldering, etc)
- [x] Software Basics (provision Duet 3 Mini 5+)
- [x] Stepper Motor Config (steps/deg, speed, accel, homing files, homing sequence)
- [x] Stall detection homing
- [x] End Effector (servo gripper installed, configured and running)
- [x] Get IK coded and set up (building custom RFF)    
- [ ] Get robo viewer plugin set up on DWC   
- [ ] Switch from custom IK to RRF once 3.5 is released
- [ ] Switch to Coral Mini Dev board (smaller, lighter, in axis camera connector)
   
   
## 🎛️ Duet DWC Config File   
``` gcode
; TODO
```
   
## 📸 Media   
TODO
