**TECHNICAL PROJECT PROPOSAL**

**Edge AMR — Autonomous Navigation with Visual-Inertial SLAM**

**on Jetson Orin Nano Super**

Ngoc Giang | Fulbright University Vietnam | June 2026

**1\. Project Overview**

Build an Autonomous Mobile Robot (AMR) that navigates real environments using camera \+ IMU-based Visual-Inertial SLAM, with GPU-accelerated inference on Jetson Orin Nano Super and real-time motor control on ESP32 via micro-ROS.

**Performance Targets**

| Metric | Target |
| :---- | :---- |
| Localization drift | \<5 cm per 5 m travel |
| Navigation success rate | ≥80% in mapped environment |
| SLAM update rate | ≥30 Hz |
| /cmd\_vel → motor response | \<20 ms |
| Motor control loop (ESP32) | 100 Hz |

 

**2\. Full System Architecture**

| \+----------------------------------------------------------+ |          	JETSON ORIN NANO SUPER                  	| |      	(Isaac ROS Docker \-- ROS2 Humble)           	| |                                                          | |  \[IMX219 CSI\] \--\> \[argus\_camera\_node\] \--\> /camera/image  | |                                                          | |  /imu \<----------------------------------------------+  | |                                                       |  | |  \[isaac\_ros\_visual\_slam\] \<-- /camera/image\_raw    	|  | |     	|         	      /imu                  	|  | |     	v                                             |  | |  /visual\_slam/tracking/odometry                   	|  | |  /map (occupancy grid)                            	|  | |     	|                                             |  | |     	v                                             |  | |  \[Nav2 Stack\]                                         |  | |	\+-- global\_planner (A\*)                        	|  | |	\+-- local\_planner (DWB)                	        |  | |	\+-- costmap\_2d                                 	|  | |     	|                                             |  | |     	v                                             |  | | 	/cmd\_vel  (geometry\_msgs/Twist)               	|  | \+---------|---------------------------------------------+  |       	| UART Serial 115200 baud                     	       	v                                                  \+------------------------------------------+           	 |      	  ESP32  (micro-ROS)         	|           	 |                                           |           	 |  Subscribes: /cmd\_vel                 	|           	 |  Publishes:  /odom  \---------------------\> Jetson     	 |          	/imu   \---------------------\> Jetson     	 |                                           |           	 |  FreeRTOS Tasks:                      	|           	 |  \+-- imu\_task: 	I2C MPU6050 @ 200 Hz   |           	 |  \+-- encoder\_task: ISR pulse count    	|           	 |  \+-- pid\_task: 	velocity PID @ 100 Hz  |           	 |  \+-- uros\_task:	micro-ROS spin     	|           	 |     	|                             	|           	 |     	v                             	|           	 |  \[TB6612FNG Motor Driver\]             	|           	 |  \+-- Motor L: PWM \+ DIR               	|           	 |  \+-- Motor R: PWM \+ DIR               	|           	 \+------------------------------------------+           	       	|           	|                              	 	\[TT Motor L\]	\[TT Motor R\]                        	 	\[LM393 enc \]	\[LM393 enc \]                        	 |
| :---- |

 

**3\. Hardware Stack**

| Component | Spec | Role |
| :---- | :---- | :---- |
| Jetson Orin Nano Super | 67 TOPS, 8 GB, JetPack 6.x | SLAM inference, navigation planning |
| IMX219 CSI camera | 8 MP, hardware ISP, 30 fps | Primary vision sensor |
| ESP32 | 240 MHz dual-core, FreeRTOS | Real-time motor control, sensor bridge |
| TT DC Motor x2 | 3–6 V, gear ratio 1:48 | Differential drive actuation |
| LM393 encoder x2 | 20-slot optical disk | Wheel velocity feedback |
| MPU6050 | 6-DOF IMU, I2C 400 kHz | Visual-inertial sensor fusion |
| TB6612FNG | Dual H-bridge, 1.2 A/channel | Motor PWM driver |
| Powerbank | 20000mAh, USB 5V 3A output | Main power source (replaces LiPo + LM2596) |

 

**4\. Software Stack**

| Layer | Technology |
| :---- | :---- |
| OS | Ubuntu 22.04 (JetPack 6.x) |
| Container | Isaac ROS Docker (CUDA 12.x) |
| Middleware | ROS2 Humble |
| SLAM | isaac\_ros\_visual\_slam (Elbrus — NVIDIA GPU-accelerated) |
| Navigation | Nav2 (navigation2) |
| Camera driver | isaac\_ros\_argus\_camera |
| MCU framework | ESP-IDF \+ FreeRTOS (not Arduino) |
| MCU ROS bridge | micro-ROS for ESP-IDF |
| Visualization | RViz2 |
| Build system | colcon |

 

**5\. ROS2 Topic Design**

| Topic | Message Type | Publisher | Subscriber |
| :---- | :---- | :---- | :---- |
| /camera/image\_raw | sensor\_msgs/Image | argus\_camera | visual\_slam |
| /camera/camera\_info | sensor\_msgs/CameraInfo | argus\_camera | visual\_slam |
| /imu | sensor\_msgs/Imu | ESP32 (micro-ROS) | visual\_slam |
| /odom | nav\_msgs/Odometry | ESP32 (micro-ROS) | Nav2 |
| /visual\_slam/tracking/odometry | nav\_msgs/Odometry | visual\_slam | Nav2 |
| /map | nav\_msgs/OccupancyGrid | visual\_slam | Nav2 costmap |
| /cmd\_vel | geometry\_msgs/Twist | Nav2 | ESP32 (micro-ROS) |
| /tf | tf2\_msgs/TFMessage | visual\_slam \+ Nav2 | All nodes |
| /goal\_pose | geometry\_msgs/PoseStamped | RViz2 / mission | Nav2 |

 

**6\. ESP32 Firmware Architecture  (ESP-IDF \+ FreeRTOS)**

**Task Structure**

| // 4 FreeRTOS tasks, pinned to cores: Task 1: imu\_task  (Core 0, 200 Hz)   \-\> I2C read MPU6050 (accel \+ gyro raw)   \-\> Pack sensor\_msgs/Imu   \-\> Push to micro-ROS publisher queue Task 2: encoder\_task  (Core 0, interrupt-driven)   \-\> GPIO interrupt on LM393 rising edge   \-\> Increment pulse counter (left / right)   \-\> Calculate RPM per wheel Task 3: pid\_task  (Core 1, 100 Hz hardware timer)   \-\> Read target velocity from /cmd\_vel   \-\> Read actual RPM from encoder\_task   \-\> PID: error \= target\_rpm \- actual\_rpm   \-\> Output: PWM duty cycle to TB6612FNG   \-\> Compute odometry (x, y, theta)   \-\> Push /odom to publisher queue Task 4: uros\_task  (Core 1\)   \-\> micro-ROS spin (subscription callbacks \+ publish queue)   \-\> Serial transport to Jetson @ 115200 baud |
| :---- |

**PID Velocity Controller**

| error(t)  \= v\_target \- v\_actual u(t)  	\= Kp \* error \+ Ki \* integral(error) \+ Kd \* d(error)/dt PWM   	\= clamp(u(t), 0, 255\) |
| :---- |

**Wheel Odometry from Encoder (20-slot disk)**

| v\_left	\= (pulses\_left  / 20\) \* wheel\_circumference / dt v\_right   \= (pulses\_right / 20\) \* wheel\_circumference / dt v\_linear  \= (v\_left \+ v\_right) / 2 v\_angular \= (v\_right \- v\_left)  / wheel\_base x 	\+= v\_linear \* cos(theta) \* dt y 	\+= v\_linear \* sin(theta) \* dt theta \+= v\_angular \* dt |
| :---- |

 

**7\. 8-Week Roadmap**

| Week | Focus | Deliverable |
| :---- | :---- | :---- |
| 1 ✅ | micro-ROS hello world | ESP32 publishes ROS2 topic visible on Jetson — DONE |
| 2 | Hardware wiring: motor driver, encoder, power | ESP32 controls TT motors, reads LM393 encoder, publishes /odom |
| 3 | IMX219 → Isaac ROS → visual\_slam | Carry Jetson around room; trajectory visible in RViz2 |
| 4 | PID velocity firmware \+ /cmd\_vel subscriber | Send Twist from ROS2; robot moves at commanded velocity accurately |
| 5 | Nav2 \+ full end-to-end pipeline | Camera → SLAM → Nav2 → cmd\_vel → motors: fully autonomous movement |
| 6 | Semantic navigation | Detect object via TensorRT; robot navigates to detected target |
| 7 | Stress test \+ metrics \+ demo | Localization error measured; success rate quantified; GitHub \+ video |
| 8 | Buffer / stretch goals | Waypoint patrol, return-to-dock, multi-session mapping |

 

**8\. Enhancement Ladder**

| Level 0  (base):    Navigate to (x, y) coordinate Level 1:        	MPU6050 visual-inertial fusion \-\> \<3 cm drift Level 2:        	Waypoint patrol route (A \-\> B \-\> C \-\> A loop) Level 3:        	Semantic: detect object via TensorRT \-\> navigate to it Level 4:        	Return-to-dock (ArUco marker on charging station) Level 5:        	Multi-session mapping (save / load map across reboots) |
| :---- |

 

**9\. Risks & Mitigations**

| Risk | Likelihood | Mitigation |
| :---- | :---- | :---- |
| Isaac ROS Docker fails on JetPack 6.x | Medium | Pin exact JetPack \+ Isaac ROS version matrix from NVIDIA compatibility table |
| Powerbank auto-shutoff during idle | Medium | Verified stays on during idle; monitor during long autonomous runs |
| micro-ROS serial latency spikes | Low–Medium | Use hardware UART (not USB-CDC); 115200 baud; ring buffer in uros\_task |
| SLAM drift \> 5 cm without IMU | Medium | Add MPU6050 to ESP32 I2C bus at Week 3 if drift measured above threshold |
| Nav2 config complexity | High | Start with single minimal YAML, tune one parameter at a time; disable unused layers |

**10\. Personal Goals**

1. Understand all layers of this project  
2. System Thinking  
3. Creative problem-solving

 

*Build first. Polish later. Ship or it didn’t happen.*  
 *Target: \<5 cm drift  |  ≥30 Hz SLAM  |  Week 8 demo.*

---

**11\. Weekly Reading List**

Rule: study one week ahead of what you’re building. Concept before math. 20 minutes max per sitting without writing 3 sentences of output.

| Week | Study This | Resource | Why |
| :---- | :---- | :---- | :---- |
| 1 (done) | micro-ROS, ESP-IDF basics | micro-ROS docs | Foundation for ESP32 ROS bridge |
| 2 | Differential drive kinematics | Articulated Robotics — "How a differential drive robot works" | Understand how 2 wheel speeds → linear + angular velocity |
| 2 | PID intuition | Phil’s Lab — "PID Controller" | Feel why Kp/Ki/Kd do what they do before tuning |
| 2 | H-bridge + PWM basics | Phil’s Lab — motor driver videos | Know what TB6612FNG is actually doing |
| 3 | ROS2 core concepts (nodes, topics, tf2) | ROS Robot Programming PDF — Ch. 1–3 (free, Robotis) | You need this for SLAM integration in Week 3 |
| 3 | Odometry drift — why it happens | Articulated Robotics — "Odometry" video | Know why /odom alone isn’t enough for navigation |
| 3 | Camera calibration basics | ROS2 camera calibration docs | IMX219 needs calibration before visual SLAM works |
| 4 | PID tuning methods | Phil’s Lab — Ziegler-Nichols video | Tune velocity PID systematically, not by guessing |
| 4 | geometry\_msgs/Twist explained | ROS2 docs — Twist message | Understand /cmd\_vel before subscribing to it |
| 5 | Nav2 architecture overview | Articulated Robotics — "Nav2" series (3 videos) | Global planner, local planner, costmap — know all 3 |
| 5 | TF2 transforms in ROS2 | Articulated Robotics — "TF2" video | Full pipeline needs correct map→odom→base\_link chain |
| 6 | TensorRT INT8 inference basics | NVIDIA TensorRT docs — Getting Started | Week 6 semantic navigation uses TensorRT |
| 6 | Object detection pipeline overview | YouTube: YOLO explained (short version) | Semantic nav needs a detector, know the pieces |
| 7 | Localization error metrics | Any mobile robotics textbook — Ch. on evaluation | Know how to measure and report your <5 cm drift claim |
| 8 | Stretch: multi-session SLAM / map saving | ROS2 map\_server docs | If reaching stretch goals in buffer week |

