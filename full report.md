# **Table of Contents**

[**Table of Contents	3**](#heading=h.wr63jp6j84i9)

[**A Camera-Based Autonomous Mobile Robot for Indoor Search and Rescue	3**](#a-camera-based-autonomous-mobile-robot-for-indoor-search-and-rescue)

[**1\. Introduction	3**](#1.-introduction)

[**1.1 What the robot does	3**](#1.1-what-the-robot-does)

[**1.2 Why we built it	4**](#1.2-why-we-built-it)

[**1.3 How to read this report	5**](#1.3-how-to-read-this-report)

[**2\. System Overview	5**](#2.-system-overview)

[**2.1 The two-computer split	5**](#2.1-the-two-computer-split)

[**2.2 How data moves through the system	6**](#2.2-how-data-moves-through-the-system)

[**2.3 Design principles	7**](#2.3-design-principles)

[**3\. Hardware and Physical Construction	7**](#3.-hardware-and-physical-construction)

[**3.1 Component list	7**](#3.1-component-list)

[**3.2 Chassis and drivetrain	8**](#3.2-chassis-and-drivetrain)

[**3.3 The stereo camera rig	9**](#3.3-the-stereo-camera-rig)

[The 45-degree mount	10](#the-45-degree-mount)

[**3.4 Electronics and wiring	10**](#3.4-electronics-and-wiring)

[**3.5 Power architecture	11**](#3.5-power-architecture)

[**4\. The Drive System (ESP32 Firmware)	12**](#4.-the-drive-system-\(esp32-firmware\))

[**4.1 What the ESP32 is responsible for	12**](#4.1-what-the-esp32-is-responsible-for)

[**4.2 Driving the motors	12**](#4.2-driving-the-motors)

[**4.3 Measuring wheel speed	13**](#4.3-measuring-wheel-speed)

[**4.4 Sensing rotation with the IMU	14**](#4.4-sensing-rotation-with-the-imu)

[Measuring the drift and cancelling it	14](#measuring-the-drift-and-cancelling-it)

[**4.5 The three control loops	15**](#4.5-the-three-control-loops)

[Loop 1: Per-wheel speed control (PID)	15](#loop-1:-per-wheel-speed-control-\(pid\))

[Loop 2: Turn rate correction	15](#loop-2:-turn-rate-correction)

[Loop 3: Heading lock	16](#loop-3:-heading-lock)

[Live tuning	16](#live-tuning)

[**4.6 Odometry: tracking position	17**](#4.6-odometry:-tracking-position)

[**4.7 Safety features	17**](#4.7-safety-features)

[**5\. Communication Between the Two Computers	18**](#5.-communication-between-the-two-computers)

[**6\. The Vision System	19**](#6.-the-vision-system)

[**6.1 Cameras and image capture	19**](#6.1-cameras-and-image-capture)

[**6.2 Detecting the target	19**](#6.2-detecting-the-target)

[The GPU problem	20](#the-gpu-problem)

[The rectification bug	20](#the-rectification-bug)

[CPU budget	20](#cpu-budget)

[**6.3 Estimating distance with two cameras	21**](#6.3-estimating-distance-with-two-cameras)

[How this evolved	21](#how-this-evolved)

[What the hardware allows	21](#what-the-hardware-allows)

[Measured accuracy	21](#measured-accuracy)

[**7\. The Mission	21**](#7.-the-mission)

[**7.1 The room model	21**](#7.1-the-room-model)

[**7.2 The patrol route	22**](#7.2-the-patrol-route)

[**7.3 Detecting and reporting a target	23**](#7.3-detecting-and-reporting-a-target)

[Reporting it usefully: directions from the start	23](#reporting-it-usefully:-directions-from-the-start)

[Measured end-to-end accuracy	23](#measured-end-to-end-accuracy)

[**8\. The Operator Dashboard	23**](#8.-the-operator-dashboard)

[**9\. Design Decisions and Deliberate Exclusions	24**](#9.-design-decisions-and-deliberate-exclusions)

[**9.1 Full path planning (Nav2) — built, not used	24**](#9.1-full-path-planning-\(nav2\)-—-built,-not-used)

[**9.2 Visual SLAM — built, not used	24**](#9.2-visual-slam-—-built,-not-used)

[**9.3 Marker-based localisation — considered, not built	25**](#9.3-marker-based-localisation-—-considered,-not-built)

[**9.4 Full-room exploration — considered, rejected	25**](#9.4-full-room-exploration-—-considered,-rejected)

[**10\. Known Limitations and Future Work	26**](#10.-known-limitations-and-future-work)

[**10.1 Position drift has no correction	26**](#10.1-position-drift-has-no-correction)

[**10.2 The robot is too heavy for its motors	26**](#10.2-the-robot-is-too-heavy-for-its-motors)

[**10.3 Direction-blind wheel sensors	27**](#10.3-direction-blind-wheel-sensors)

[**10.4 Distance accuracy degrades with range	27**](#10.4-distance-accuracy-degrades-with-range)

[The dense method we built, and why it did not work	27](#the-dense-method-we-built,-and-why-it-did-not-work)

[How this compares to the field	27](#how-this-compares-to-the-field)

[Where the research frontier is	27](#where-the-research-frontier-is)

[The cheaper improvement	27](#the-cheaper-improvement)

[**10.5 Heading lock engages while stationary	27**](#10.5-heading-lock-engages-while-stationary)

[**10.6 Full-loop verification	27**](#10.6-full-loop-verification)

[**11\. Conclusion	28**](#11.-conclusion)

[**Appendix A — Pin Map	28**](#appendix-a-—-pin-map)

[**Appendix B — Key System Parameters	29**](#appendix-b-—-key-system-parameters)

[Physical	29](#physical)

[Stereo calibration	29](#stereo-calibration)

[Control gains (defaults)	30](#control-gains-\(defaults\))

[Mission	30](#mission)

[**Appendix C — Repository Structure	31**](#appendix-c-—-repository-structure)

[**Appendix D — External References	31**](#appendix-d-—-external-references)

[**Appendix E — Measurement Summary	32**](#appendix-e-—-measurement-summary)

**\[NOTE\]** The entries above are complete and correctly named for this revision, but the **page numbers are stale** — roughly 3,300 words were added across Sections 6.3, 7.1, 7.2, 7.3, 10.4, 10.6 and the appendices, so everything after Section 6 has shifted. Regenerate the table of contents in Google Docs before submission rather than trusting these numbers.

# **Quack Quack**

# **A Camera-Based Autonomous Mobile Robot for Indoor Search and Rescue**

**System Description Report**

Ngoc Giang — Fulbright University Vietnam June – August 2026  
Project repository: `github.com/ChuoiUhuhu0727/slam-amr`**\[NOTE — Title page\]** Suggested layout: robot name and subtitle centred, then a full-width hero photo of the finished robot, then author/date/repo block at the bottom. Best photo for this: the Week 6 shot showing the stereo camera rig, the 3D-printed mount, and the 18650 battery, or a cleaner final-build photo if one exists.**\[PHOTO — Figure 0\]** *Caption: "Quack Quack" in its final build. The two cameras sit on a rigid 3D-printed mount, angled toward the room's interior. The Jetson Orin Nano Super sits behind them; the ESP32 and motor driver are on the protoboard below.*

# 

# **1\. Introduction**

# **1.1 What the robot does**

We built a small wheeled robot that searches a room on its own and reports what it finds.

The robot is called "Quack Quack." It does one job. It drives a loop around the inside edge of a room. While it drives, it looks inward toward the middle of the room with two cameras. It is watching for a specific target object. In our tests, that target is a rubber duck.

When the robot sees the target, it works out how far away the target is and in which direction. It combines that with its own position to estimate where the target actually sits in the room. Then it reports that position back to an operator on a live web page.

The rubber duck is a stand-in for whatever the robot is really looking for. In a real deployment that would be a person, or a marker, or a piece of equipment. The duck is simply an object we could train a detector on quickly, place anywhere in a test room, and photograph from every angle without difficulty. Nothing about the rest of the system depends on the target being a duck.

The robot does all of this without a lidar, and without any outside help — no GPS, no network, no map given to it beyond the dimensions of the room, and nobody driving it.

# **1.2 Why we built it**

This robot is designed for search and rescue and reconnaissance: entering a space that is unsafe for a person, and reporting what is inside it.

After an earthquake, a fire, or a chemical spill, the first question responders need answered is a question about position — is anyone in there, and where? Answering it normally means sending someone in. A robot that goes instead, identifies what it finds, and reports coordinates back is doing useful work even if it does nothing else.

Two requirements follow from that use case, and they shaped the whole build.

**The robot has to be cheap enough to risk losing.** A robot sent into a damaged structure may not come back out. That rules out lidar — the spinning laser scanner most autonomous robots use to measure distance — because a single industrial unit costs more than our entire machine. We used ordinary camera modules instead, at roughly USD 20 each. The trade-off is real: a camera measures distance less accurately than a lidar, and gets worse with range. The question this project asks is how much of the job you can still do anyway.

**The robot cannot depend on anything outside itself.** No GPS reaches inside a collapsed building. There may be no lighting, no network, no map, and nobody available to drive it. Whatever the robot knows about its own position, it has to work out from its own sensors. This is why so much of this report deals with position tracking, particularly Sections 4.4 to 4.6. A robot that finds a target but cannot say where has not helped.

The same capability transfers to safer settings — a robot that patrols a known space, recognises objects, and reports their positions also works as a stocktaking or security patrol robot, where low cost matters too. But those are secondary. The design target throughout was the dangerous room.

The goal was never a finished product. It was an honest working prototype, and a clear account of where a camera-only approach holds up and where it breaks down.

# **1.3 How to read this report**

This is a system description report. It describes the finished system: what it is made of, how the pieces fit together, and why we built each piece the way we did.

It is written to be read on its own. You do not need to have read the weekly progress reports first.

The report moves from the outside in. Section 2 gives the whole system in one view. Section 3 covers the physical robot: the parts, the chassis, the wiring, the power. Sections 4 through 8 go through each subsystem in turn, roughly in order from the lowest level (spinning a motor) to the highest (reporting a target on a map). Sections 9 and 10 are honest accounting: what we chose not to build and why, and what still does not work well.

Technical terms are explained in plain language the first time they appear.

**This report does not cover performance.** How accurately the robot drives and measures, how often it completes a mission, and how it compares to other robots all belong to the companion **System Evaluation Report**. This report covers what the system *is* and why it is built that way; that one measures what it *does*.

# **2\. System Overview**

# **2.1 The two-computer split**

The robot runs on two computers, not one. They do very different jobs.

**The NVIDIA Jetson Orin Nano Super** is the brain. It is a small single-board computer with a built-in graphics processor (GPU) designed for running AI models. It handles everything that requires thinking: capturing camera images, running the object detection model, estimating distance to the target, deciding where to drive next, and serving the operator dashboard.

**The ESP32** is the reflex system. It is a cheap microcontroller — a very small computer with no operating system in the usual sense. It handles everything that has to happen precisely on time: reading the wheel sensors, reading the motion sensor, running the motor control loops, and tracking the robot's position.

The split exists because these two jobs conflict. Running an AI vision model is slow and unpredictable — one frame might take 80 milliseconds, the next might take 200\. Controlling a motor needs the opposite: a steady heartbeat that never skips. If both jobs shared one processor, the vision work would occasionally starve the motor loop, and the robot would twitch or drift.

Giving the motor control its own dedicated chip removes that risk entirely. The ESP32 does not care how busy the Jetson is.

We hit exactly this conflict in a smaller form even after splitting the system. Vision processing on the Jetson was competing with the Jetson's own navigation loop for CPU time. We fixed it by moving detection onto its own thread and capping how often it runs. That is covered in Section 6\.

# **2.2 How data moves through the system**

The full loop, from sensing to moving, works like this:

1. The two cameras capture images. The Jetson reads both.

2. The detection model looks for the target in each image.

3. If the target is found in both images, the Jetson triangulates its distance and direction.

4. Separately, the Jetson's mission logic decides where the robot should drive next.

5. That decision becomes a simple speed command: how fast to go forward, and how fast to turn.

6. The command travels to the ESP32 over a USB cable.

7. The ESP32 turns that command into actual motor power, correcting continuously using its sensors.

8. The ESP32 sends its position estimate back to the Jetson.

9. The Jetson uses that position, plus the target's measured distance and direction, to place the target on a map.

10. Everything appears live on a web dashboard.

**\[DIAGRAM — Figure 1\]** *Caption: System architecture. The Jetson (top) handles vision, mission logic, and the dashboard. The ESP32 (bottom) handles real-time motor control and position tracking. They exchange commands and position estimates over a single USB serial link.***\[NOTE\]** You already have a version of this diagram from Week 2, but it is now out of date — it shows Isaac ROS visual SLAM and Nav2 as live parts of the pipeline, and neither is used in the final mission. I would redraw it. Suggested boxes, top to bottom: **Jetson** → \[Two IMX219 cameras\] → \[YOLOv8n detection\] → \[Stereo distance\] → \[Mission logic\] → \[Flask dashboard\]; then the link `/cmd_vel` down and `/odom` up; then **ESP32** → \[Heading control\] → \[Per-wheel PID\] → \[TB6612FNG driver\] → \[Two TT motors\], with \[LM393 encoders\] and \[MPU6050 IMU\] feeding back up into the control block.

# **2.3 Design principles**

Three ideas shaped most of our decisions.

**Separate the real-time work from the slow work.** This is the two-computer split above. It is the single most important structural decision in the project.

**Make it observable.** Early on we debugged by reading raw text logs scrolling past in a terminal. It was slow and it hid problems. Partway through the project we built a live web dashboard showing the robot's position, both camera feeds, detections, and internal control values. Nearly every bug we found after that point was found faster because of it.

**Prefer a simple thing that works over a sophisticated thing that might.** Several times we had a more advanced option available and chose the simpler one. The clearest example is navigation. We had a full path-planning stack wired up and working, but the final mission runs on a fixed four-corner route instead. Section 9 explains why.

# **3\. Hardware and Physical Construction**

# **3.1 Component list**

| Component | Part | Role |
| :---- | :---- | :---- |
| Main computer | NVIDIA Jetson Orin Nano Super (8 GB) | Vision, mission logic, dashboard |
| Microcontroller | ESP32 (dev board) | Real-time motor control, sensor reading, odometry |
| Cameras | 2 × IMX219 CSI camera modules | Stereo image capture |
| Motor driver | TB6612FNG dual H-bridge | Converts control signals into motor power |
| Motors | 2 × TT geared DC motors | Drive the two wheels |
| Wheel sensors | 2 × LM393 optical encoders (20 slots) | Measure wheel rotation |
| Motion sensor | MPU6050 (6-DOF IMU) | Measure rotation rate and orientation |
| Compute power | USB-C PD power bank (15V) | Powers the Jetson and ESP32 |
| Motor power | 18650 LiPo cell + boost (9V) | Powers the motors only |
| Chassis | Person | Physical frame |

**\[NOTE\]** Two things to verify and fill in here. First, the exact chassis/kit you used — I did not want to guess the model. Second, whether the third contact point is a caster wheel or a ball caster; the photos are not conclusive. Both matter for the drivetrain description below.**\[PHOTO — Figure 2\]** *Caption: The main components before assembly. \[Or: use the Week 3 "Early protoboard" photo here — it shows the ESP32, the TB6612FNG driver board in red, and both LM393 encoder boards clearly.\]*

# **3.2 Chassis and drivetrain**

The robot uses **differential drive**. That means it has two powered wheels, one on each side, and it steers by driving them at different speeds. Both wheels forward at the same speed drives straight. Left wheel forward and right wheel backward spins the robot in place. There is no steering mechanism at all — the speed difference is the steering.

This is the simplest practical drive layout, and it is what almost every small indoor robot uses. It also turns in place, which matters for us: our test room is only 1 metre wide on its short axis, so a robot that needed a turning circle would not fit the patrol path.

The physical numbers that the software depends on:

* **Wheel diameter: 67 mm.** The firmware uses this to convert wheel rotation into distance travelled. Getting it wrong scales every distance the robot reports.

* **Wheelbase (distance between the two wheels): 100 mm.** This converts a speed difference between wheels into a turn rate.

Both of these are measured physical values, and both are hardcoded into the firmware. We got the wheel diameter wrong at first, and every distance the robot reported was proportionally wrong until we re-measured and corrected it.**\[NOTE — suggested reflection, please confirm\]** This is worth calling out as a lesson: two hand-measured constants sit underneath every position number the robot produces. No amount of good control code compensates for a mis-measured wheel. Confirm this framing matches how it actually felt at the time before keeping it.

The robot ended up heavier than we originally planned for. The Jetson, the power bank, the battery, the camera rig, and the enclosure all added up. The TT motors are sized for a lighter platform. This single fact caused a chain of problems later — stalling on turns, aggressive control tuning to compensate, and heat in the power converter. Section 10.2 covers the consequences.**\[PHOTO — Figure 3\]** *Caption: The assembled chassis from underneath, showing the two drive motors, the wheels, and the encoder discs. \[The Week 3 "Mounted enclosure" photo showing both LM393 boards mounted to the chassis plate would work well here.\]*

# **3.3 The stereo camera rig**

Two cameras are what replace the lidar. The technique is **stereo vision**, and it works the way human eyes do. Hold a finger up and close one eye, then the other: the finger appears to jump sideways, a lot when it is close and barely at all when it is far. That sideways jump is called **disparity**, and it is a direct measurement of distance.

For the robot to use this, two physical facts must be exact and unchanging:

**The separation between the cameras must be known precisely.** We used 8.3 cm. To hold that spacing we 3D printed a mounting bar with screw holes exactly 8.3 cm apart. Our first print was too thin — the camera screws protruded through the back and made the bar bow upward, which tilted both cameras. We reprinted it thicker so it sat perfectly flat.

**The two cameras must not move relative to each other, ever.** They are mounted rigidly as one unit. If one shifts by even a couple of millimetres, every distance measurement afterwards is wrong, and nothing in the images reveals it.

Both are build problems rather than software problems, and both bit us. The bowing mount was one. The other was subtler: our calibration software computed the separation as 10.1 cm instead of 8.3 cm — a 22% error. We spent real time trying to fix it in software, including a more complex lens distortion model. The cause was a slightly loose camera ribbon cable. Reseating it fixed the maths instantly.

## **The 45-degree mount**

The cameras do not point forward. They are rotated **45 degrees toward the robot's right side.**

This comes directly from the mission. While the robot hugs the perimeter, its forward direction points *along the wall*, not into the room. A forward-facing camera would spend the whole patrol staring down a wall and rarely see the middle of the room. Angling the rig inward fixes this: the robot drives along the wall while looking into the interior.

This mounting angle also dictates which way the robot must drive the loop. With the cameras looking to the right, the room's interior only stays on the robot's right if the robot walks the loop **clockwise**. We had this backwards at first — the patrol ran counter-clockwise, which pointed the cameras out through the wall for the entire run. The robot completed its route perfectly and saw nothing.**\[NOTE — suggested reflection, please confirm\]** This is a good "insight" candidate: a bug where every subsystem worked correctly and the mission still failed, because a physical mounting angle and a route direction disagreed. Nothing in the code was wrong. Confirm the story as I have told it.**\[PHOTO — Figure 4\]** *Caption: The stereo camera rig. Both IMX219 modules are mounted to a single 3D-printed bar with screw holes exactly 8.3 cm apart, and the whole rig is angled 45° toward the robot's right so it looks into the room while driving along a wall.***\[PHOTO — Figure 5, optional\]** *Caption: The first and second versions of the 3D-printed camera mount. The thinner first version bowed upward around the camera screws; the thicker version sits flat.* — only if you photographed both.

# **3.4 Electronics and wiring**

The ESP32, the motor driver, the encoders, and the IMU are all mounted on a protoboard — a perforated board where connections are made by soldering wires by hand.

The ESP32's connections are listed in full in Appendix A. In summary:

* Six pins go to the motor driver: two direction pins and one speed pin per motor, plus one shared enable pin.

* Two pins read the wheel encoders.

* Two pins carry the I2C bus that talks to the motion sensor. I2C is a two-wire standard for connecting small sensor chips to a controller.

The wiring gave us one of the project's more frustrating early failures. Connections were intermittently unreliable even though every solder joint looked correct on inspection. The cause turned out to be the wire itself — the specific wire we were using did not bond properly with our solder, producing joints that looked perfect and conducted badly. We resoldered with a different wire type over the originals and the problem stopped.

Around the same time, our ESP32 board turned out to ship with a non-standard crystal frequency, which produced garbled serial output and a boot loop until we corrected it in the SDK configuration.

Neither problem was visible by inspection. Both only appeared under test.**\[PHOTO — Figure 6\]** *Caption: The protoboard carrying the ESP32, the TB6612FNG motor driver (red board), and the two LM393 encoder boards.***\[NOTE\]** If you still have the wiring diagram you made in Week 3, it belongs here as a figure. It is more useful to a new reader than a photo of the board.

# **3.5 Power architecture**

Power is split into two completely separate supplies that never touch each other.

**Supply 1 — compute.** A USB Power Delivery power bank outputs 15 V to the Jetson. The Jetson's official adapter is 19.5 V, but the board accepts a range, and 15 V from a PD power bank is clean and convenient. The ESP32 draws its 5 V from the Jetson over the USB cable that also carries their data link.

**Supply 2 — motors.** A single 18650 lithium cell feeds a boost converter, which steps its voltage up to 9 V for the motor driver. Nothing else runs off this supply.

The separation is deliberate and it solved a real problem. Before we split them, the system suffered brownouts — the ESP32 would reset at random, and the whole system would freeze unpredictably.

The cause is that DC motors are electrically violent. When a motor starts, stalls, or reverses, it pulls a large sudden spike of current and dumps electrical noise back into whatever it is connected to. If sensitive electronics share that supply, the voltage sags briefly and they reset.

Splitting the supplies means the motors can misbehave as much as they like without the Jetson or the ESP32 noticing. After the split, the resets stopped completely.

The motor supply was originally 5 V. We raised it to 9 V later, when the robot's weight started causing motors to stall on turns. That helped but did not fully solve it, and the boost converter now runs hot because the motors sit near their maximum current draw for long stretches. This is discussed further in Section 10.2.**\[DIAGRAM — Figure 7\]** *Caption: Power architecture. Two isolated supplies. The power bank feeds the Jetson at 15 V and the ESP32 at 5 V over USB. A separate 18650 cell and boost converter feed the motors at 9 V. Keeping motor current off the compute supply eliminated the brownout resets.***\[NOTE\]** A simple two-branch block diagram is enough here. This is one of the clearest "we diagnosed a real problem and fixed it structurally" stories in the project, so a diagram earns its space.

# **4\. The Drive System (ESP32 Firmware)**

# **4.1 What the ESP32 is responsible for**

The ESP32 runs the firmware in `esp32/motor_f1/`. It is built on FreeRTOS, a small real-time operating system that lets several jobs run on a schedule without interfering with one another.

Two jobs run concurrently:

* **The control task** runs every 50 milliseconds, or 20 times per second. It reads the sensors, runs all the control maths, and sets the motor power.

* **The communication task** handles the link to the Jetson, and publishes the robot's position estimate 20 times per second.

Everything in this section happens on the ESP32, with no involvement from the Jetson. The Jetson only ever says "drive at this speed, turn at this rate." How that actually gets achieved is entirely the ESP32's problem.

# **4.2 Driving the motors**

The ESP32 cannot power a motor directly — its pins deliver tiny currents. The TB6612FNG motor driver sits in between. It is an **H-bridge**: a switching circuit that takes a low-power control signal and uses it to route battery power through a motor in either direction.

Each motor gets three signals:

* **Two direction pins**, which set whether the motor spins forward or backward.

* **One speed pin**, carrying a PWM signal.

**PWM** stands for pulse-width modulation. Rather than varying the voltage, the controller switches full power on and off very rapidly and varies the ratio between on and off. Hence, this allows for motor speed control. 

# **4.3 Measuring wheel speed**

Each wheel has an **LM393 slotted optical encoder**. A disc with 20 evenly spaced slots is attached to the wheel. A light beam shines across the disc, and a sensor detects each time a slot passes. Twenty pulses means one full wheel revolution.

The ESP32 measures the time between pulses to compute rotation speed, then converts to distance using the wheel diameter.

These encoders have an important limitation: they are **single-channel**, which means they are direction-blind. They can tell you the wheel is turning and how fast, but not which way. The firmware assumes the wheel is turning in whichever direction it was last commanded to turn. This is fine in normal operation and wrong if a wheel is pushed backwards or slips against the commanded direction. A two-channel (quadrature) encoder would remove this assumption.

The encoder signal also needed cleaning. Early on, the robot reported wheel speeds above 6000 RPM — physically impossible for these motors. The cause was electrical noise: the brushes inside a spinning DC motor spark, and that noise reached the encoder pins and was counted as extra pulses.

The fix was a software filter that ignores any pulse arriving less than 15 milliseconds after the previous one. Anything faster than that is not a real slot passing; it is noise. This is a good example of a broader rule we relearned repeatedly: clean sensor data before making decisions with it.

# **4.4 Sensing rotation with the IMU**

The **IMU** (inertial measurement unit) is an MPU6050 chip. It contains two sensors:

* A **gyroscope**, which measures how fast the chip is rotating.

* An **accelerometer**, which measures acceleration, including gravity.

The gyroscope is the important one for us. It tells the robot how fast it is turning, dozens of times per second, entirely independently of the wheels. That independence is the whole point. Wheels slip; the gyroscope does not care.

A gyroscope measures *rate* of rotation, not direction faced. To get a heading — which way the robot is pointing — the firmware adds up the rotation rate over time. This is called integration.

Integration has a well-known weakness: **drift**. Every reading carries a tiny error, and adding up thousands of tiny errors accumulates into a growing one. A gyroscope left completely still will slowly report that it has rotated.

## **Measuring the drift and cancelling it**

Our answer to drift is a **calibration period at startup**, and it is deliberately simple.

The insight is that most of a gyroscope's drift is not random. It is a constant offset. A gyroscope sitting perfectly still does not report zero — it reports some small fixed value, and that value is what accumulates into drift. If we can measure that resting value, we can subtract it from every reading afterwards, and the accumulation largely stops.

So when the ESP32 boots, it does exactly that. It takes 400 gyroscope readings roughly 10 milliseconds apart, with the robot held completely still, and averages them. That takes about **four seconds**. The resulting average is the sensor's resting bias. Every reading from that point forward has it subtracted.

Averaging many samples matters, because each individual reading is noisy. Noise falls away with the square root of the number of samples, so taking four times as many readings gives roughly half the noise in the final number. Four seconds of boot time is a cheap price for a measurably better heading estimate over the whole run.

This does place one requirement on the operator: **the robot must be completely still while it calibrates.** If it is moved, bumped, or still settling during those four seconds, the measured bias is wrong, and the error is then baked into every heading reading for the rest of the mission. Sitting the robot down and letting it finish booting before starting a run is part of the operating procedure, not an optional courtesy.**\[NOTE\]** Worth saying plainly to a reader: this is a software fix for a hardware limitation, and it does not eliminate drift, it reduces it. The remaining drift is what Section 10.1 is about.

# **4.5 The three control loops**

This is the heart of the drive system, and it is the part of the project that changed most.

The Jetson sends the robot a simple instruction: a forward speed and a turn rate. Three nested control loops turn that instruction into motor power.

## **Loop 1: Per-wheel speed control (PID)**

Each wheel has its own **PID controller**. A PID controller is a standard feedback loop. It compares where you want to be against where you actually are, and adjusts the output based on that error.

The name comes from three terms:

* **P (proportional)** — push harder the further off you are. Our value: 3.0.

* **I (integral)** — if a small error persists, keep building up correction until it goes away. This is what overcomes steady resistance like friction or a slight incline. Our value: 0.2, capped so it cannot build up without limit.

* **D (derivative)** — react to how fast the error is changing. We do not use a D term.

Each wheel runs its own independent loop with its own accumulated correction. This matters. An earlier design used one shared loop averaging both wheels, which we reverted — a shared loop couples the speed correction and the steering correction together through one output, so tuning one degrades the other.

## **Loop 2: Turn rate correction**

The second loop compares the turn rate the Jetson asked for against the turn rate the gyroscope actually measures. If the robot is turning too slowly, it nudges the wheel speed targets apart; too fast, and it brings them together.

Note the mechanism: this loop does not change the PID gains. It shifts each wheel's **target speed**, and the speed loops then do their normal job of hitting those targets. Keeping steering and speed correction separate this way is why the shared-PID design had to go.

## **Loop 3: Heading lock**

The third loop is the one that made the drive system trustworthy, and it was the biggest single redesign in the project.

The problem it solves: the turn rate loop only works while the robot is actively turning. Once rotation stops, the measured turn rate is zero, the error is zero, and the loop stops correcting — even if the robot is now pointing in completely the wrong direction. It can hold a *turn rate* but it cannot hold a *direction*.

The heading lock fixes this. The moment the robot is not being commanded to turn, it latches its current heading as a target and holds it. Any drift away from that heading — from a wheel slipping, from an uneven floor, from momentum carrying it past the end of a turn — produces a correction that steers it back.

In practice this is the difference between "the robot mostly goes forward" and "the robot goes where you pointed it." Before the heading lock, the robot zigzagged and drifted, because nothing tied the two wheels' behaviour to an actual direction. After it, the robot holds a straight line and self-corrects for slip.

The original design derived heading from the wheel encoders alone. Encoders cannot detect slip by definition — if a wheel spins without gripping, the encoder happily reports movement that never happened. Moving heading onto the gyroscope removed that blind spot.

## **Live tuning**

All of the gains for these three loops can be changed from the dashboard while the robot is running, without recompiling or reflashing the firmware. Nine values are adjustable: the three speed-loop terms, two turn-rate terms, two heading-lock terms, and a small trim value per wheel to compensate for the two motors not being identical.

This was one of the highest-value things we built. Before it, changing one number meant a full recompile-and-flash cycle. After it, tuning became something we could actually iterate on in seconds. The default values are listed in Appendix B.**\[DIAGRAM — Figure 8\]** *Caption: The three control loops. The Jetson's speed and turn commands enter at the left. The heading-lock and turn-rate loops adjust each wheel's target speed, then each wheel's own PID loop converts its target into motor power. The gyroscope feeds both heading loops; the wheel encoders feed the speed loops.***\[NOTE\]** This diagram is worth real effort — it is the most technically substantial part of the report and the hardest to follow in prose. Your Week 2 PID pipeline diagram is the right visual style, it just needs updating to show all three loops and the gyroscope feedback path.

# **4.6 Odometry: tracking position**

**Odometry** is the robot estimating its own position by keeping a running tally of its own movement. Every control cycle, it asks: how far did each wheel turn, and how much did I rotate? It adds that step to its running estimate of x, y, and heading.

This is **dead reckoning** — position from accumulated movement, with no external reference. It is how a ship navigated before satellites. It works, and it drifts, and the drift only ever grows. Nothing in our system ever corrects the robot's x and y position against an outside truth.

One improvement worth noting. When the robot moves and turns during the same cycle, using the heading from the start of the step or the end of the step both introduce error. We use the average of the two instead, a technique called **midpoint integration**. This measurably improved position accuracy during turns and cost nothing but a slightly different line of arithmetic.

The heading component of the estimate is in much better shape than the position component. Heading has an independent sensor checking it — the gyroscope, which does not care whether the wheels are slipping. Position has nothing. It is built entirely from wheel rotation, and any error in it stays there permanently.

This asymmetry is the single biggest structural limitation in the finished system, and Section 10.1 discusses what could be done about it.

# **4.7 Safety features**

Two safety mechanisms are built into the firmware.

**Command timeout.** If the ESP32 does not receive a new command from the Jetson for one second, it stops the motors. This means a crashed program, an unplugged cable, or a hung process results in a stopped robot rather than a robot that keeps driving with its last instruction.

**Motor kill switch.** The dashboard has a control that pulls the motor driver's standby pin low, cutting power to the driver outputs. This is a hardware-level cutoff. It is deliberately different from commanding zero speed: at zero speed the control loops are still running and can still react to sensor noise with small twitches of power. The kill switch removes power entirely.

We added the kill switch after live testing made the need obvious. Before it, the only way to cut motor power was to comment out a line of firmware and reflash.

# **5\. Communication Between the Two Computers**

The Jetson and the ESP32 talk over a single USB cable using **micro-ROS**.

ROS 2 (Robot Operating System 2\) is the standard framework for robot software. Its core idea is **topics**: named channels that programs publish messages to and subscribe to, without needing to know anything about each other. micro-ROS is a cut-down version that runs on microcontrollers like the ESP32.

A program on the Jetson called the micro-ROS agent bridges between the two. The ESP32 speaks micro-ROS over the serial cable; the agent translates it into normal ROS 2 topics that the rest of the Jetson's software can use.

| Topic | Direction | Purpose |
| :---- | :---- | :---- |
| `/cmd_vel` | Jetson → ESP32 | Forward speed and turn rate command |
| `/odom` | ESP32 → Jetson | Estimated position and heading (20 Hz) |
| `/imu` | ESP32 → Jetson | Raw gyroscope and accelerometer readings |
| `/pid_gains` | Jetson → ESP32 | Live control gains, kill switch, and reset commands |
| `/esp32_diag` | ESP32 → Jetson | Diagnostics: speeds, PWM, error, reset reasons |

Two details worth noting.

`/cmd_vel` uses the standard ROS message type for velocity commands. Using the standard type rather than inventing our own is why we were able to plug a full off-the-shelf navigation stack into this robot without modifying the firmware.

`/pid_gains` carries more than gains. It is a general text-command channel — tuning values, the motor kill and resume commands, and the odometry reset all travel over it. We reused one channel rather than adding new ones for each feature, which kept the firmware simpler.

The `/esp32_diag` channel deserves specific mention. It publishes the robot's internal state — what each wheel is actually doing, what power is being applied, how far off the heading is, and why the ESP32 last restarted. This last item answered a question that was otherwise very hard to answer: when the robot froze, had it browned out, or had the software crashed? Without that, we would have guessed.

# **6\. The Vision System**

# **6.1 Cameras and image capture**

The two IMX219 modules connect directly to the Jetson over CSI ribbon cables. CSI is a dedicated high-speed camera interface, faster and lower-latency than USB. Each camera captures at 1280×720 at 30 frames per second.

Getting the cameras working at all took some diagnosis. Our first test used NVIDIA's standard `nvgstcapture-1.0` tool, which crashed without capturing a single frame. We initially suspected a broken ribbon cable. The real cause was further along: the camera was capturing correctly, and the crash happened when the tool tried to compress the frames into H.264 video. Our pipeline never needs video compression — we want raw frames for the detector — so we bypassed the tool entirely.

# **6.2 Detecting the target**

Detection uses **YOLOv8n**, a small object-detection neural network. The "n" is for nano — the smallest variant in the family, chosen because it runs fast enough for live use on the Jetson.

The model was not trained from scratch. We **fine-tuned** it: starting from a model already trained on general objects and teaching it one new specific thing.

Our first dataset was built by hand. We filmed the rubber duck from many angles with the Jetson's own camera, extracted frames from the video, and trained on those. Filming with the robot's own camera was deliberate — the training images then matched the robot's real viewing angle, the duck's apparent size at realistic distances, and the colour rendering of these particular camera modules.

That first model worked, with some false positives. We later retrained on a larger public dataset once the GPU was running properly.

## **The GPU problem**

For a long stretch of the project, detection was quietly running on the Jetson's CPU instead of its GPU.

Nothing appeared broken. The pipeline ran, the detections were correct, and no errors appeared. It was just slow — which is exactly why it took so long to notice. A wrong answer is obvious; a right answer arriving slowly just looks like a heavy model.

Once fixed, detection was fast enough to run live during a patrol.**\[NOTE — for the evaluation report\]** The before/after accuracy numbers for the retrained model, and the before/after frame rates for the GPU fix, belong in the System Evaluation Report rather than here. Neither is in the weekly reports yet, so both still need collecting.

## **The rectification bug**

This bug is worth describing because it is easy to make and hard to see.

Camera lenses distort images, bending straight lines near the edges. Correcting that is called **rectification**, and the stereo distance maths needs rectified images to be accurate.

So when we built the stereo pipeline, we rectified the images and fed them to the detector. Detection accuracy dropped. The reason: our model was trained entirely on raw, uncorrected images, and rectified ones look subtly different — different enough that the model was seeing a kind of image it had never encountered in training.

The fix was to run detection on raw frames and apply the rectification maths only to the resulting coordinates. Detection sees what it was trained on; the distance maths still gets its corrected geometry.

## **CPU budget**

Vision competes with navigation for the Jetson's processor. Three changes reduced the pressure: detection runs on its own thread rather than blocking the control loop, the second camera is skipped when the first camera sees nothing, and the detection rate is capped at 5 Hz so vision cannot consume every spare cycle.

# **6.3 Estimating distance with two cameras**

The current method works as follows. Both cameras detect the duck independently, each producing a bounding box. The system takes the horizontal centre of each box and subtracts one from the other. That difference is the disparity — the sideways jump described in Section 3.3.

Distance then comes from a standard relationship: distance is proportional to the camera separation divided by the disparity. Big disparity means close; small disparity means far.

Two sanity checks guard the result. A disparity below 2 pixels is rejected, because at that point the reading is either meaningless or the two cameras have matched different objects entirely. Any computed distance beyond 5 metres is also rejected, since our room's diagonal is under 3 metres.

Direction is simpler: where the duck appears horizontally in the frame gives its angle relative to the camera, and the camera's fixed 45° mounting offset converts that into an angle relative to the robot.

## **How this evolved**

Our first version used **one** camera and assumed the duck's real-world height was already known. From a known height and an apparent size in pixels, distance follows. This works, and it only works for objects whose size you have told the system in advance. It cannot generalise to an unknown target.

Moving to two-camera triangulation removed that assumption. The robot now measures distance from geometry alone, without knowing anything about the object's size. For a system meant to find things other than rubber ducks — a person in a collapsed room is not a known size — that is the more important property, and we accepted a harder measurement problem to get it.

## **What the hardware allows**

Before quoting any accuracy figure it is worth establishing what this particular camera pair can physically achieve, because that sets the floor everything else is measured against.

Distance from disparity follows Z = fx · B / d, where fx is the focal length in pixels and B the camera separation. Differentiating gives the error a single pixel of disparity costs: ΔZ = Z² / (fx · B) · Δd. Our calibration file gives fx = 875.51 pixels and B = 85.40 mm, so fx · B = 74.77.

| True distance | Disparity | Error from 1 pixel | As % of distance |
| :---- | :---- | :---- | :---- |
| 0.30 m | 249 px | 1.2 mm | 0.40 % |
| 0.50 m | 149 px | 3.3 mm | 0.67 % |
| 1.00 m | 75 px | 13.4 mm | 1.34 % |
| 1.30 m | 57 px | 22.6 mm | 1.74 % |
| 1.50 m | 50 px | 30.1 mm | 2.01 % |

The consequence is that error growth with distance is not a defect to be fixed. It is the geometry of the method. Any accuracy claim about this system is meaningless unless the distance it was measured at is stated alongside it.

## **Measured accuracy**

Measured against a tape measure with the robot stationary and the duck static at each distance, reading the smoothed value off the dashboard:

| Ground truth | Measured | Error |
| :---- | :---- | :---- |
| 0.30 m | 0.34 m | +13 % |
| 0.50 m | 0.54–0.59 m | +9 to +18 % |
| 1.00 m | 1.10 m | +10 % |
| 1.30 m | 1.30 m | ~1 % |

Two honest observations about this table. First, the system reads consistently **long** between 0.3 m and 1.0 m — it reports the duck as further away than it is, by roughly a tenth of the true distance. Second, the 1.30 m reading is far better than the rest, and we do not have an explanation for why. A near-constant percentage error is the signature of a scale problem in the calibration, but a scale error would not spare one distance and not the others.

We therefore describe this system as accurate to roughly 10 % between 0.3 m and 1.0 m, and note the 1.30 m result as an unexplained outlier rather than as the headline figure. Each distance above represents a single placement, not a repeated trial, so no statement about spread or repeatability can be made from this data.**\[NOTE\]** If you get time for one more experiment before submission, repeating each distance five times and reporting mean and standard deviation would convert this table from three anecdotes into a defensible curve.


# **7\. The Mission**

# **7.1 The room model**

The robot does not build a map. The room is described to it in advance, as two numbers: width and length.

Our demo room is **2.23 m × 1.0 m**, physically measured with a tape measure. The long axis is the x axis, because x is defined as the direction the robot faces when it starts.

The coordinate origin is a **corner of the room**, and the robot starts at a known offset from it — 30 cm out from the wall behind it, 25 cm out from the wall on its left, facing straight down the long axis. Odometry still reports zero wherever the robot is switched on, so that fixed offset is added to every reading to convert into room coordinates.

This has a real practical consequence: **the robot must be placed correctly at the start of every run.** There is no way for it to work out that it has been placed in the wrong corner or facing the wrong way. Its entire sense of place is anchored to that one assumption. The orientation matters far more than the position: a few degrees of yaw error at the start rotates the whole coordinate frame permanently, and at 1.7 m range five degrees of it displaces a reported target by about 15 cm. A practical way to place the robot squarely is to measure from the side wall to the front and rear of the chassis and make the two readings equal, which is far more precise than judging it by eye.

For reporting, the room is divided into a grid of 0.5 m cells — a 4×2 grid for this room. A target is reported by grid cell rather than as a raw coordinate. This is an honest presentation choice: reporting "cell B3" rather than "1.42 m, 0.87 m" communicates roughly the precision the system actually has, instead of implying centimetre accuracy it does not have.

# **7.2 The patrol route**

The robot is placed **already standing on the route and already aligned down the long straight**, so the mission opens with a straight leg and then makes nothing but 90-degree right turns:

| Leg | Action | Distance |
| :---- | :---- | :---- |
| 1 | Drive straight down the long wall | 163 cm |
| 2 | Turn right 90°, cross the short end | 50 cm |
| 3 | Turn right 90°, drive back down the far long wall | 163 cm |
| 4 | Turn right 90°, cross back to the start | 50 cm |

This starting arrangement was a deliberate change made during testing, and it fixed a real problem. The robot originally started facing down the room's *short* wall, which meant its first action was an in-place turn toward the far end. Any heading error left by that opening turn was then integrated over the 2.23 m straight that immediately followed, and with odometry alone nothing ever corrects it. That was the cause of the robot drifting into walls. Starting aligned removes the blind opening turn entirely.

The insets from the walls are set per axis rather than as one shared number, because the two axes do different jobs. Along the long axis the inset is **0.30 m**, which is braking room before the end walls and gives the 163 cm straights. Across the short axis it is **0.25 m**, which sets the **50 cm width of the patrol loop** — the corridor the camera sweeps between. Widening that from an earlier 40 cm improved coverage at the cost of reducing side-wall clearance from 30 cm to 25 cm.

The loop runs **clockwise**, for the camera-mounting reason given in Section 3.3. Every leg keeps the room's interior on the robot's right, which is where the camera rig looks.

Returning to the starting waypoint at the end is not only tidiness. Because the robot started there, and believes it is there again, the difference between where it thinks it is and where it physically ended up is a direct measurement of accumulated drift over a full loop.

To drive between waypoints, a simple controller computes the direction to the next waypoint and steers toward it. If the required turn is large — more than about 50 degrees — the robot stops and turns in place first, rather than trying to arc around. Arriving within 10 cm counts as reaching the waypoint. Speeds are capped at 0.30 m/s forward and 1.5 rad/s turning, both deliberately conservative for a room this small.**\[DIAGRAM — Figure 9\]** *Caption: The patrol route. The robot starts already aligned down the 2.23 m straight, 30 cm from the end wall and 25 cm from the side wall on its left, and drives a clockwise loop of 163 cm, 50 cm, 163 cm, 50 cm so its right-angled camera rig faces the room's interior throughout. The target sits near the centre.***\[NOTE\]** A simple top-down rectangle with four dots, arrows showing the clockwise direction, and a small cone showing the camera's 45° view angle would communicate more than anything else in this section.

# **7.3 Detecting and reporting a target**

When both cameras detect the duck at the same time, the robot combines four things: its own estimated position, its own estimated heading, the measured distance to the duck, and the measured bearing to the duck. Those give the duck's position in room coordinates.

That position is then reported as a grid cell on the dashboard.

Note that the target's reported position inherits every error in the chain. If the robot's own position estimate has drifted 8 cm, the target's reported position is off by at least that much before the distance measurement's own error is added on top. The measurements compound.

## **Reporting it usefully: directions from the start**

A room coordinate is exact but not actionable. Nobody standing in a doorway knows where the room's origin is. The one place a person can reliably stand and orient themselves is the spot they put the robot down, because they put it there.

The system therefore converts the target's room coordinate into a distance and a bearing measured from the robot's start pose and its starting heading: turn this many degrees, walk this far. The dashboard shows this live, and also draws it as a line on the map, so a sign error in the bearing shows up as an arrow pointing at the wrong wall rather than hiding inside a number.

One property of this is worth stating because it is not obvious. The start pose is added to the odometry reading to get room coordinates, and then subtracted again to get the distance from the start. It cancels exactly. That means an error in where the robot was physically placed does **not** affect the reported directions — they are measured from wherever the robot actually started. A *rotational* placement error does not cancel, because the gyroscope zeroes its heading at whatever angle the robot was set down at, and nothing ever corrects it. Placing the robot straight matters; placing it in exactly the right spot does not.

## **Measured end-to-end accuracy**

This is the number the whole system exists to produce, and the only one that includes every layer at once. Measured with the robot **driving**, against a tape measure from the start position to the target:

| True distance from start | Reported | Error |
| :---- | :---- | :---- |
| 0.31 m | 0.30 m | 1 cm |
| 0.90 m | 0.74 m | 16 cm |
| 1.04 m | 1.00 m | 4 cm |

Two of the three placements land within 1–4 cm. For a robot localising on wheel odometry alone, with no laser scanner and no SLAM, using passive stereo cameras in ambient light, that is a result we are comfortable defending — and even the 16 cm outlier would put a searcher close enough to see the target immediately.

It should be read for exactly what it is. This measurement stacks four error sources on top of each other: the stereo distance, the bearing angle, the robot's odometry position and heading accumulated over the drive, and the running average over every sighting in the run. Three placements cannot separate them. It is strong evidence that the *system* works end to end, and no evidence at all about any individual component's accuracy.**\[NOTE\]** Section 6.3's table is the component-level measurement; this one is the system-level result. Keeping the two clearly apart is what stops the report from over-claiming.

One known weakness sits inside this number. The running average that produces the final answer is a plain mean over every sighting recorded during the run, with no outlier rejection — while the live distance readout beside it uses a median, chosen specifically to throw out badly-localised detections. A single bad sighting that passes the sanity checks therefore biases the final answer permanently. This is the most likely explanation for the 16 cm outlier above, and it is a small fix we have identified but not yet made.

# **8\. The Operator Dashboard**

The Jetson runs a small web server. Any device on the same network can open the dashboard in a browser.

It shows:

* **A live map** of the room, with the robot's current position, the patrol waypoints, and any detected target.

* **Both camera feeds**, live, with detection boxes drawn on them.

* **The control panel** with all nine tunable gains, editable and applied instantly.

* **A manual drive tester** — commands like "drive 1 metre" or "turn 90 degrees," run independently of the mission.

* **Diagnostics** — each wheel's actual speed, the power being applied, and the current heading error.

* **Mission controls** — start, stop, reset position, and the motor kill switch.

The dashboard was not in the original plan. We built it because reading raw terminal logs was making debugging slow.

It repaid the effort several times over. Two features in particular changed how we worked. Live gain tuning turned control tuning from a recompile-and-reflash guessing game into fast iteration. The manual drive tester let us isolate one movement at a time — drive exactly 1.8 m, then measure with a tape where the robot actually stopped — which is the basis of the measurements reported in the System Evaluation Report.**\[NOTE — suggested reflection, please confirm\]** There is a general lesson here worth stating: the tool for observing the system turned out to be as important as the system itself. Almost none of the accuracy testing that the evaluation report draws on would have been practical without it. Confirm this matches your experience.**\[PHOTO — Figure 10\]** *Caption: The operator dashboard during a patrol run. Left: the live room map with the robot's estimated position and the patrol route. Centre: both camera feeds with detections. Right: the live tuning panel and diagnostics.***\[NOTE\]** A screenshot here is genuinely important — the dashboard is a major deliverable and it is the only part of the system a reader can't picture from a photo of the robot. A screenshot mid-run, with a duck detected, would be ideal.

# **9\. Design Decisions and Deliberate Exclusions**

Several capabilities were built and then not used, or considered and not built. Listing them matters, because a reader who knows robotics will look for them and should know they were decisions rather than oversights.

# **9.1 Full path planning (Nav2) — built, not used**

We implemented Nav2, the standard ROS 2 navigation stack. It plans paths, avoids obstacles using a live map of what the cameras see, and drives to a goal clicked on a screen. We got it working, including feeding stereo depth data in as an obstacle map.

The final mission does not use it. It drives four fixed waypoints instead.

The reasoning: the room is small, known in advance, and empty. Nav2's value is planning routes through complex or changing spaces. Our route has four corners and no obstacles. Meanwhile Nav2 is a large, complex system with many failure modes, and it was the least-tested part of our stack. Close to the deadline, we chose the simple thing we could fully verify over the sophisticated thing we could not.

# **9.2 Visual SLAM — built, not used**

**SLAM** stands for Simultaneous Localisation and Mapping: a robot building a map of an unknown space while working out its own position within it. This was in the project's original title and original plan.

We got NVIDIA's GPU-accelerated Isaac ROS visual SLAM running on the Jetson, publishing pose estimates at a stable 30 Hz.

We then spent a multi-day debugging session on a bug where it exaggerated the robot's movement by three to five times. We methodically ruled out frame timing and camera geometry configuration before discovering the tracker was breaking permanently whenever the robot was physically bumped or pushed during testing — the resulting motion blur and frame jumps were unrecoverable. That led to a strict hands-off testing rule.

The scale bug was never fully resolved. With the mission scoped to a known room, SLAM's core benefit — mapping unknown space — was not needed, so we deferred it rather than continue debugging.

This is worth being direct about: the project's name still says SLAM, and the finished system does not use SLAM. That is a genuine change in direction, and the reasons are the ones above.

# **9.3 Marker-based localisation — considered, not built**

Placing printed markers (ArUco tags or QR codes) at known positions is a well-established, cheap fix for drift. The robot sees a marker, knows where it is, and corrects its position against it. This would have directly addressed our biggest weakness. We did not build it, for time.

It is worth noting where this solution applies, because it sits awkwardly against our use case. Markers must be placed in advance by someone who already knows the space — nobody is putting up QR codes inside a collapsed building. Markers are the right tool for development and testing, where they give us ground truth to measure drift against, and for the prepared spaces mentioned in Section 1.2.

For the dangerous-room case, the robot needs to correct against features already present: measuring to a known wall, or ultimately visual SLAM. Both are discussed in Section 10.1.

# **9.4 Full-room exploration — considered, rejected**

We considered having the robot cover the entire room rather than just its perimeter. This requires frontier exploration — the robot deciding for itself where it has not looked yet and going there, while mapping as it goes.

We rejected it. It depends on the mapping capability we had already deferred, and reopening that close to the deadline risked breaking a demo that already worked.**\[NOTE — suggested reflection, please confirm\]** These four decisions share a shape: as the deadline approached, the highest-value engineering decision was repeatedly to build less. There is a real lesson in that worth stating explicitly, since a reader might otherwise read this section as a list of failures rather than a list of choices. Confirm you are happy with that framing.

# **10\. Known Limitations and Future Work**

# **10.1 Position drift has no correction**

The robot's x and y position comes purely from dead reckoning. It has no way to check that estimate against anything external. Heading has the gyroscope checking it; position has nothing.

Errors therefore only accumulate. Over a short run this is manageable. Over a longer mission or a larger space it would not be, and a robot that reports a target in the wrong place is arguably worse than one that reports nothing.

**Possible fixes, in increasing order of effort:**

* **Markers at known positions** (Section 9.3). Cheapest and most direct, but only usable in a prepared space.

* **Measure to a known wall.** The stereo cameras can already measure distance to a flat surface. Comparing that against where a wall is expected gives a position correction.

* **Full visual SLAM.** Most capable, most complex. Blocked on the scale bug in Section 9.2.

# **10.2 The robot is too heavy for its motors**

This is the most consequential hardware limitation, and it caused a chain of downstream problems.

The robot ended up heavier than the TT motors were designed to move. Driving straight is fine, since both motors share the load. Turning in place loads each motor individually, and on uneven floor a motor can stall completely.

Our workaround was in software. Instead of ramping motor power up smoothly, we tuned the control loop to fire a large burst of power immediately, which overcomes static friction. This works, but results in a visible side-to-side jiggle as the robot drives.

Real options for a root-cause fix:

* **Add a large capacitor (around 1000 µF) to the motor supply**, to buffer high-current bursts.

* **Move from a single-cell 9 V boost setup to a two-cell 12 V supply.**

* **Reduce weight, or re-gear for more torque.**

**\[NOTE — status\]** As of writing, this decision is still open — the capacitor versus 12 V question has not been resolved or tested. If you settle it before submission, this section needs the result. One thing to plan for: **moving to 12 V will very likely require re-tuning the control gains**, probably making them less aggressive, since the same commanded power produces more torque at a higher voltage.

# **10.3 Direction-blind wheel sensors**

The encoders detect that a wheel is turning but not which way. The firmware assumes the commanded direction. Quadrature (two-channel) encoders would remove this assumption for a small cost increase.

# **10.4 Distance accuracy degrades with range**

Stereo distance measurement is inherently least accurate at long range, for the reason set out in Section 6.3: distance is inversely proportional to disparity, so at range a fixed pixel error in the detection box translates into a much larger distance error. At 0.3 m one pixel of disparity is worth 0.4 % of the distance; at 1.5 m the same pixel is worth 2.0 %.

## **The dense method we built, and why it did not work**

The obvious improvement is to compute depth across many points on the object instead of one, and average them. We built exactly this: dense stereo matching across the whole image, reprojected into a 3D point cloud, taking the median of the points falling inside the detected box. Averaging thousands of points should reject the jitter that a single measurement cannot.

Below roughly 0.5 m it worked, reaching 6-10 % error. Beyond that it failed, and failed in an unusual way: at a true distance of 1.3 m it returned three distinct, repeating clusters of values - around 0.4 m, around 1.5 m, and around 2.4 m - while the simple single-point method held steady at 1.30 m using the exact same detection box.

The cause is not a tuning problem. The duck's surface is smooth and almost featureless. Dense stereo matches small image patches between the two cameras, and a patch of untextured yellow plastic looks identical to every other patch of untextured yellow plastic, so the algorithm cannot say where it is. The only pixels it can match confidently are the textured floor and wall visible at the edges of the box, so it locks onto the background and reports the background's distance. We confirmed this directly: at 1.3 m the eroded centre region of the box produced zero confidently-matched points on every failing frame, while the full box always had over a hundred, all near the edges. Enlarging the centre region was tried and made the readings noisier, not better, which confirms the mechanism is missing texture rather than an unlucky crop.

We shipped a reliability fix rather than an accuracy one: if the dense method disagrees with the single-point method by more than 30 %, its distance is rescaled onto the single-point value. This caps how wrong any one reading can be. It does not make the dense method more accurate.

## **How this compares to the field**

This failure is not a flaw peculiar to our implementation. It is the known, structural limitation of passive dense stereo, and it is worth stating how the field handles it, because that reframes what we built.

NVIDIA's own production stereo depth package for this exact hardware family, Isaac ROS DNN Stereo Depth (ESS), lists the same limitation in its documentation: disparity for highly reflective and textureless surfaces is not reliably measured. A rubber duck is a textureless surface. A commercially deployed, deep-learned stereo model running on the same class of Jetson has the same failure mode we hit.

The industry's answer is not better software but extra hardware. The Intel RealSense D435, the usual reference point for a depth camera good enough for a robot, quotes better than 2 % depth error at 2 m from a 50 mm baseline, narrower than our 85 mm. It achieves that with an **infrared pattern projector** that throws artificial texture onto the scene, which exists precisely so that smooth, untextured objects become matchable. Our dense method's failure is the exact problem that projector was invented to solve.

| | This robot | Intel RealSense D435 |
| :---- | :---- | :---- |
| Camera separation | 85 mm | 50 mm |
| Depth method | Passive stereo, ambient light | Active IR stereo with pattern projector |
| Useful range | ~0.3-1.5 m (measured) | 0.3-3 m (specified) |
| Accuracy | ~10 % at 0.3-1.0 m | Better than 2 % at 2 m |

Read that comparison as intended: a purpose-built depth camera with active illumination outperforms two USD 20 camera modules, which is the expected result. What the comparison establishes is that our gap comes from a missing hardware capability, not from a mistake in the geometry or the code.

## **Where the research frontier is**

The strongest current work on this problem is **FoundationStereo** (Wen et al., NVIDIA, CVPR 2025, Oral presentation and Best Paper Nomination), a foundation model for stereo matching trained on one million synthetic stereo pairs and designed for zero-shot generalisation, meaning no fine-tuning on the scene it is deployed in. It holds first place on both the Middlebury and ETH3D stereo leaderboards.

Two things make it the natural next step rather than merely a citation. It officially supports the Jetson Orin, with an ONNX and TensorRT deployment path, so it runs on the computer we already have. And a real-time variant, Fast-FoundationStereo, reports better than a tenfold speed-up at close to the same accuracy, which matters directly because our own dense stereo takes about one second per frame on this Jetson and is therefore rate-limited to roughly 1 Hz.

One caution about quoting stereo research alongside our numbers. Stereo-matching papers are almost universally evaluated with a metric called **bad-2.0**: the percentage of *pixels* in a dense disparity map whose disparity is wrong by more than 2 pixels, on a fixed public dataset. That is not our metric. Ours is the percentage error in the *metric distance to one detected object*. The two are not comparable, and a direct "our 10 % versus their X %" claim would be a methodological error. The honest way to relate them is through the table in Section 6.3: a method achieving half a pixel of disparity accuracy would yield roughly 0.9 % distance error *on our rig*, which is what a state-of-the-art matcher would actually buy us.

## **The cheaper improvement**

Beyond software, the single most effective change available to us is mechanical: **a wider camera baseline.** Distance error scales inversely with camera separation, so moving the two cameras from 85 mm to, say, 170 mm apart would halve the error at every range, at the cost of a new mount and a recalibration. For a system operating between 0.3 m and 1.5 m this is likely a better return on effort than any change to the algorithm.

# **10.5 Heading lock engages while stationary**

The heading lock activates from the moment the firmware boots, before the robot has ever been commanded to move. Vibration or gyroscope drift while sitting still can therefore trigger small corrective motor pulses.

The fix is to require at least one real movement command before the lock arms. This is known and not yet implemented.

# **10.6 Full-loop verification**

The system has now been run end to end with the robot driving and reporting target positions, producing the measurements in Section 7.3: the target's position relative to the start pose was correct to within 1-4 cm on two of three placements. That is the headline result of the project, and it is real data from a moving robot rather than a bench test.

What remains unverified is narrower, but worth stating plainly:

* **Repeatability.** Every accuracy figure in this report comes from a single placement at each distance. Nothing has been measured five times and reported as a mean with a spread, so no claim about consistency can be made.

* **Odometry drift on its own.** The patrol route returns to its own starting point, so the position error reported on arrival is a direct measurement of accumulated drift over one loop. This costs nothing extra to record and has not yet been captured.

* **The bearing sign.** The conversion from a target's horizontal pixel position into an angle relative to the robot has never been checked against a target placed deliberately to one known side. The code has carried a note to this effect since it was written. If the sign were inverted, distances would stay correct while reported positions were mirrored.

* **The dense-stereo gate.** The 30 % disagreement gate described in Section 10.4 is deployed but has not been observed triggering correctly across the full range.

**\[NOTE\]** This section previously said no end-to-end run had been completed. That is now out of date, since the run happened. Rewritten to claim the result and list what genuinely remains. The three unverified items above are each well under an hour of work if you want to close any of them before submission.

# **11\. Conclusion**

We set out to build a robot that could enter a room unsafe for a person, work out where it was, find what it was looking for, and report where that thing was — using USD 20 cameras instead of a laser scanner costing more than the rest of the machine combined.

The finished robot does all four. It drives a perimeter route on its own, holds a straight line, and turns to a commanded angle, correcting itself when a wheel slips. It watches the room's interior with two inward-angled cameras, recognises the target it was trained on, works out the distance from geometry alone, and reports a position on a live map an operator can read from another room.

The architecture is the part we would defend most strongly. Splitting the system across two computers — one for thinking, one for reflexes — removed an entire category of problem before it could appear. The heading lock, built on the gyroscope rather than the wheels, is the single change that turned an unreliable drivetrain into a foundation the rest of the system could sit on. The dashboard, never in the original plan, became the tool that made everything else debuggable.

One engineering lesson recurred more than any other: almost every serious problem lived in the physical and electrical layer, not in the algorithms.**\[NOTE — please confirm\]** The conclusion is the most interpretive section, since it synthesises rather than reports. Everything factual traces back to the weekly reports or the code, but the emphasis is my reading. Adjust so it sounds like you.

# **Appendix A — Pin Map**

ESP32 connections. Motor channel A drives the left wheel; channel B drives the right.

| ESP32 Pin | Connects to | Function |
| :---- | :---- | :---- |
| GPIO 21 | TB6612FNG AIN1 | Left motor direction 1 |
| GPIO 22 | TB6612FNG AIN2 | Left motor direction 2 |
| GPIO 17 | TB6612FNG PWMA | Left motor speed (PWM) |
| GPIO 18 | TB6612FNG BIN1 | Right motor direction 1 |
| GPIO 19 | TB6612FNG BIN2 | Right motor direction 2 |
| GPIO 16 | TB6612FNG PWMB | Right motor speed (PWM) |
| GPIO 23 | TB6612FNG STBY | Driver enable / hardware kill |
| GPIO 35 | Left LM393 encoder | Left wheel pulse input |
| GPIO 34 | Right LM393 encoder | Right wheel pulse input |
| GPIO 26 | MPU6050 SDA | I2C data |
| GPIO 25 | MPU6050 SCL | I2C clock |

I2C address: MPU6050 at `0x68`.

# **Appendix B — Key System Parameters**

## **Physical**

| Parameter | Value |
| :---- | :---- |
| Wheel diameter | 67 mm |
| Wheelbase | 100 mm |
| Encoder resolution | 20 slots per wheel revolution |
| Stereo camera baseline | 8.3 cm measured, 8.54 cm from calibration |
| Camera mounting angle | 45° toward the robot's right |
| Camera resolution | 1280 × 720 at 30 fps |

## **Stereo calibration**

| Parameter | Value |
| :---- | :---- |
| Rectified focal length (fx) | 875.51 px |
| Calibrated baseline | 85.40 mm |
| Calibration reprojection error | ~0.33 px |
| fx × baseline (sets depth resolution) | 74.77 |
| Minimum accepted disparity | 2 px |
| Maximum accepted distance | 5 m |

## **Control gains (defaults)**

| Gain | Value | Controls |
| :---- | :---- | :---- |
| Kp | 3.0 | Wheel speed, proportional term |
| Ki | 0.2 | Wheel speed, integral term |
| Max I | 40.0 | Cap on accumulated integral correction |
| Khead | 15.0 | Turn rate correction strength |
| Max head trim | 20.0 | Cap on turn rate correction |
| Klock | 2.0 | Heading lock strength |
| Max lock | 0.6 | Cap on heading lock correction |
| Trim L / Trim R | 0.0 / 0.0 | Per-wheel compensation |

**\[NOTE\]** These are the firmware defaults. If your final tuned values from the tuning sessions are different, replace them.

## **Mission**

| Parameter | Value |
| :---- | :---- |
| Room dimensions | 2.23 m (long axis, x) × 1.0 m (short axis, y) |
| Start pose | 30 cm from the wall behind, 25 cm from the wall on the robot's left, facing down the long axis |
| Waypoint inset, long axis | 0.30 m (gives two 1.63 m straights) |
| Waypoint inset, short axis | 0.25 m (gives a 50 cm loop width) |
| Patrol route | 163 cm, right 90°, 50 cm, right 90°, 163 cm, right 90°, 50 cm |
| Reporting grid cell size | 0.5 m (4 × 2 grid) |
| Waypoint arrival tolerance | 10 cm |
| Maximum forward speed | 0.30 m/s |
| Maximum turn rate | 1.5 rad/s |
| Distance smoothing window | median of last 8 readings |
| Dense-stereo disagreement gate | 30 % |

**\[NOTE\]** The route and start pose above were changed part-way through testing. The robot originally started facing down the room's *short* wall, which meant the mission opened with a blind in-place turn whose heading error then compounded over the long straight that followed - this was the cause of the robot drifting into walls. It now starts already aligned down the long straight, so the run opens with a straight leg and makes only right turns. If Section 7.2 still describes the old route, it needs updating to match.

# **Appendix C — Repository Structure**

slam-amr/

├── esp32/

│   └── motor\_f1/main/motor\_f1.c Drive firmware: control loops,

│       encoders, IMU, odometry, micro-ROS

├── jetson/

│   ├── mission/

│   │   └── search\_and\_rescue.py Mission logic, vision, dashboard

│   ├── calibration/ Stereo camera calibration tools

│   ├── training/ Duck detector training

│   ├── dataset\_collection/ Dataset capture and frame extraction

│   ├── nav2/ Path planning stack (built, not used)

│   ├── slam/ Visual SLAM launch files (built, not used)

│   └── tools/ Diagnostic loggers

├── duck\_dataset/ Training images and labels

└── README.md

# **Appendix D — External References**

Sources cited in Sections 6.3 and 10.4, where this system's measured accuracy is placed against published work.

1. B. Wen, M. Trepte, J. Aribido, J. Kautz, O. Gallo and S. Birchfield, "FoundationStereo: Zero-Shot Stereo Matching," *Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition (CVPR)*, 2025. Oral presentation, Best Paper Nomination. NVIDIA Labs. Available: https://arxiv.org/abs/2501.09898 — project page https://nvlabs.github.io/FoundationStereo/, code https://github.com/NVlabs/FoundationStereo

2. NVIDIA, "Fast-FoundationStereo," 2025. Real-time variant of the above, reporting better than a tenfold speed-up at close to the same zero-shot accuracy. Available: https://nvlabs.github.io/Fast-FoundationStereo/

3. NVIDIA, "Isaac ROS DNN Stereo Depth (ESS)," Isaac ROS documentation. Cited for its stated limitation that disparity for highly reflective and textureless surfaces is not reliably measured. Available: https://nvidia-isaac-ros.github.io/concepts/stereo_depth/ess/index.html

4. Intel Corporation, "Intel RealSense Depth Camera D435 — Product Specifications." Cited for the quoted depth accuracy of better than 2 % at 2 m and the 50 mm baseline. Available: https://www.intel.com/content/www/us/en/products/sku/128255/intel-realsense-depth-camera-d435/specifications.html

5. D. Scharstein et al., "Middlebury Stereo Evaluation — Version 3." Cited for the definition of the *bad-2.0* metric used throughout the stereo-matching literature: the percentage of pixels whose disparity error exceeds 2 pixels. Available: https://vision.middlebury.edu/stereo/eval3/

**\[NOTE\]** Reference 4 was taken from Intel's published specification as reported in secondary sources; confirm the exact wording against the current datasheet before submission, since the figure is quoted directly in Section 10.4.

# **Appendix E — Measurement Summary**

Every accuracy figure in this report, collected in one place with the conditions under which it was taken.

| Measurement | Range | Result | Conditions |
| :---- | :---- | :---- | :---- |
| Stereo distance, single-point | 0.30–1.00 m | ~10 % (reads long) | Robot stationary, one placement per distance |
| Stereo distance, single-point | 1.30 m | ~1 % | Robot stationary, unexplained outlier |
| Dense stereo (point cloud) | below 0.5 m | 6–10 % | Robot stationary |
| Dense stereo (point cloud) | above 0.5 m | Unusable | Locks onto background; now gated |
| **End-to-end target position** | **0.31–1.04 m** | **1–4 cm on two of three placements, 16 cm on the third** | **Robot driving** |
| Theoretical floor, 1 px disparity | 1.30 m | 1.74 % | Geometry of this rig |
| *Intel RealSense D435 (reference)* | *2.0 m* | *better than 2 %* | *Active IR stereo, datasheet* |

The distinction between rows four and five is the one to keep clear when presenting this work. The stereo rows measure a *component*. The end-to-end row measures the *system*, and is the result the project should be judged on.

**\[NOTE — final\]** Two things I deliberately did not write, because I did not have the material:

1. **A total build cost.** Section 1.2 argues the robot has to be cheap enough to risk losing. A single figure would land that argument hard.

2. **An acknowledgements section**, if the format expects one — your teammate vịt owned the vision side.

