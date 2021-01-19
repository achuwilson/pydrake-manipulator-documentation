# pydrake-manipulator-docs

This document serves as a quick introduction to Kuka IIWA Robot and controlling it (or any other manipulator) using Drake Python API

## **Contents**
- [Introduction](#introduction)
- [Kuka System Architecture](#kuka-system-architecture)
- [Programming the Robot](#programming-the-robot)
    - [Kuka Sunrise WorkBench](#kuka-sunrise-workbench)
    - [FRI](#fri)
- [Controlling IIWA from ROS](#controlling-iiwa-from-ros) 
- [Drake Concepts](#drake-concepts)
- [Drake drivers](#)
    - [Java App for Sunrise](#)
    - [kuka_driver LCM](#)
-  [LCM](#lcm)
    - [LCM Introduction](#)
    - [IIWA-LCM Interface](#)
 - [Drake Intro](#)
- [Robot hardware Interface/Drake-LCM Interface](#)
    - [Loading a custom robot model](#)
    - [Reading from hardware](#)
    - [writing to hardware](#)
- [Joint Control](#)
- [Visualizing the robot state in Drake visualizers](#)
- [Adding an end effector to the model](#)
- [Forward Kinematics](#)
- [Inverse Kinematics](#)
- [Estimating Cartesian Velocities](#)
- [Cartesian velocity control](#)
- [Estimating Cartesian forces](#)
- [Hybrid Force-Position control](#)
- [Motion Planning -  Generating Trajectories](#)

## **Introduction**


Kuka LBR IIWA is a collaborative robot manipulator which has got excellent torque control capabilities in addition to the default position control features. This enables capabilities like impedence control which is much benefitial when the  robot has to interact with noisy environment models, where pure position control can break things (or even the robot)

## **Kuka System Architecture** 
The Kuka sunrise cabinet controller has an industrial PC running  Kuka's version of Windows CE called Sunrise OS and a realtime OS. The sunrise OS handles the user program, GUI etc and is accessible to the user. The realtime part is hidden from the user and controls the low level motor drivers and other hardware interfaces. 

Kuka Smartpad, the handheld controller is just a system which shows the remote desktop view of the above mentioned Sunrise OS. Conecting an external monitor to the DVI port on the back side of the controller box shows the same Smartpad GUI. It is also possible to access the Smartpad GUI using Remote Desktop tools. The login credentials for the remote desktop are:

IP address: ```172.31.1.147``` \
Username:  ```KukaUser```\
password: ```68kuka1secpw59```


The user often connects to the robot over an Ethernet interface to pogram/control the robot. Additional interfaces like EtherCat, profinet are also available. 


## **Programming the Robot**

Kuka basically provides the following two methods to program the robot
 - Kuka Sunrise Workbench
 - FRI

### **Kuka Sunrise Workbench**
 The default programming option provided by Kuka is through its Java APIs using an IDE called Sunrise Workbench which is infact a customized Eclipse IDE. It is not available for download on any Kuka websites, as it has to match the version of the Sunrise OS running on the controller, So request for your copy of Sunrise Workbench to your Kuka robot supplier
 
 The Java APIs may differ slightly epending upon the version of the Sunrise OS and Workbench that is being used. It is available in the  (TODO -find name) document. A sample is available here. It also provides information on configuing the robot

 After developing an application in the Sunrise Workbench, the user has to synchromize it with the Sunrise OS in the controller. This just copies the project files to the controller over an ethernet port (Port X66 - Kuka Line Interface - KLI ). The default IP addess of the KLI port is ```172.31.1.147```  and a static IP must be set for the customer PC. 
 
 While creating a new project and synchronizing it with the Controller, if you changed any safety related settings, then the SmartPad would show a safety configuration not activated error. The default password to activate the safety configuration is ```ARGUS```


After loading the  applications, the desired one has to be selected and executed using the 
 Smartpad interface. 
 
### **FRI**  
  
  FRI stands for "Fast Research Interface", which is an addon provided by Kuka, which enables real time control of the robot system at the lowest level possible. This requires control signals be generated in an external computer and sent over a specific Ethernet port called KONI -  Kuka Optional Network Interface. The FRI is not enabled out of the box and has to be installed and enabled through the Sunrise workbench. The default IP address of the FRI interface is ```192.170.10.2```

  Kuka provides FRI-Client libararies in C++ and Java, which can be found inside the examples directory after the installation of FRI library in Sunrise WorkBench. The C++ libraries can be found in the file named ```FRI-Client-SDK_Cpp.zip```. It can be used to build applications talks with Kuka controller over FRI.

  Drake uses the FRI interface to control the IIWA from an external computer.  

## **Controlling IIWA from ROS**

The [```iiwa_stack```](https://github.com/IFL-CAMP/iiwa_stack) package can be used to interface IIWA fron ROS. It uses the Smart Servoing functionality over the KLI network interface. 

The ROSJava nodes running on the robot as a Sunrise RobotApplication sends data and receives commands from a ROS master running on the external PC. The [wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki) provides detailed instructions on getting ROS workinggit@github.com:achuwilson/pydrake-manipulator-docs.giter```
### **Drake IIWA Java Application**
 The Java application has to be compiled with the Kuka Sunrise Workbench and uploaded to the robot. The application opens an FRI connection to which the ```kuka_driver``` running on an external computer connects to.

 There are two Java applications
  - DrakeFRIPositionDriver 
  - DrakeFRITorqueDriver.

  The DrakeFRIPositionDriver as the name  implies allows controlling the robot in position control mode, taking in joint position commands. 

  The DrakeFRITorqueDriver allows for the control of the robot in impedance control mode and takes in joint position as well as joint feedforward torque comands

  Both the drivers output robot status like joint positions, velocities, torques etc

  
### **kuka_driver** 
The ```kuka_driver``` runs on the external computer, connects to the Java application running on the robot and provides an LCM interface to read/write data.

It has to be compiled as in this [documentation](https://github.com/RobotLocomotion/drake-iiwa-driver/blob/master/README.md) and requires FRI client SDK for compilation.

After compilation, the ```kuka_driver``` should be run in background 
## **LCM**

[LCM](https://lcm-proj.github.io/) stands for Lightweight Communications and Marshalling. It is a set of libraries that can provide publish/subscribe message passing capabilities for different applications.

### **IIWA-LCM Interface**
```kuka_driver``` provides read/write interface to the IIWA thriugh LCM messages. It generates three LCM messsage channels
 - ```IIWA_STATUS``` of the type ```lcmt_iiwa_status```, defined in [```lcmt_iiwa_status.lcm```](https://github.com/RobotLocomotion/drake/blob/master/lcmtypes/lcmt_iiwa_status.lcm)
 - ```IIWA_COMMAND``` of the type ```lcmt_iiwa_command```, defined in [```lcmt_iiwa_command.lcm```](https://github.com/RobotLocomotion/drake/blob/master/lcmtypes/lcmt_iiwa_command.lcm)
 - ```IIWA_STATUS_TELEMETRY``` of the type ```lcmt_iiwa_status_telemetry```, defined in [```lcmt_iiwa_status_telemetry.lcm```](https://github.com/RobotLocomotion/drake/blob/master/lcmtypes/lcmt_iiwa_status_telemetry.lcm)

By default, ```kuka_driver``` publishes/ subscribes these messages at 200Hz

```IIWA_STATUS``` provides the robot joint status which includes joint position, velocities and torques. An example which subscribes to the ```IIWA_STATUS``` and prints the output is available in [```lcm_examples/iiwa-lcm-listener.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/lcm_examples/iiwa-lcm-listener.py)

```IIWA_COMMAND``` is used to command joint positions with an optional feed forward joint torque. An example which subscribes to ```IIWA_STATUS``` to estimate the current robot configuration and move joint 7 incrementally is available in [```lcm_examples/iiwa-lcm-publisher.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/lcm_examples/iiwa-lcm-publisher.py)

```IIWA_STATUS_TELEMETRY``` provides timing information, which can be used to estimate the latency in the FRI communication between the external computer and the robot controller.

### **Custom Robot - LCM Interface**
In order to add a custom robot interface, we have to first create message type definitions specific to the robot configuration. This would involve the status message to get feedback from the robot and the  command message to issue control commands to the robot.

Once we have defined the custom message types, we have to create a driver, which will communicate with the robot hardware(such as through serial port/ethernet etc) and exchange the data with the LCM system as LCM messages

## **LCM - Drake Interface**
Drake contains the following systems for communicating with LCM

- ```LcmInterfaceSystem```
- ```LcmSubscriberSystem```
- ```LcmPublisherSystem```

The ```LcmInterfaceSystem``` has no inputs nor outputs nor state nor parameters; it declares only an update event that pumps LCM messages into their subscribers if the LCM stack has message(s) waiting. The subscribers will then update their outputs using their own declared events

The ```LcmSubscriberSystem``` subscribes to the LCM data stream and outputs the recived data through a single output port

The ```LCMPublisherSystem``` has a single input port and outputs the received data to the LCM data stream at a specified update rate.

### **IIWA-Drake Interface** 

The IIWA - Drake hardware interface consists of two systems
- ```IiwaStatusReceiver``` defined in [```iiwa_status_receiver.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/iiwa_status_receiver.py)
- ```IiwaCommandSender``` defined in  [```iiwa_command_sender.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/iiwa_command_sender.py)


The ```IiwaStatusReceiver``` has a single input, which has to be  connected to the output of ```LcmSubscriberSystem``` and has the following 7 vector valued outputs:

        - position_commanded
        - position_measured
        - velocity_estimated
        - torque_commanded
        - torque_measured
        - torque_external

The ```IiwaCommandSender``` has a single output which has to be connected to the input of the ```LcmPublisherSystem```. It has the following two inputs

        - position
        - torque
### **Custom Robot - Drake Interface**

Similar to the IIWA example, systems that parse the LCM message and provide inputs/outputs to the Drake systems have to be implemented

## **Manipulation Station**
TODO
### **IIWA Manipulation station**
### **Custom Manipulation station**
## **Joint Control**
[```example_joint_slider.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_joint_slider.py)

In the [```example_joint_slider.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_joint_slider.py) example, we make use of the drake ```JointSliders``` system to control the joint values of the robot. The output port of the ```JointSliders``` system is connected through a ```FirstOrderLowPassFilter``` to the ```iiwa_position``` port of the ```IiwaHardwareInterface``` manipulation station.


## **Visualizing the robot state in Drake visualizers**
[```example_iiwa_visualize.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_iiwa_visualize.py)

Drake has multiple visualizers and uses the SceneGraph method to output the visualizations. By default, Drake comes with a VTK based visualizer which is located in ```/opt/drake/bin/drake-visualizer```. We have to launch the viuslizer before running the simulation.

Drake also has a Meshcat based visualizer which can display the output in a browser window.

In the [```example_iiwa_visualize.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_iiwa_visualize.py) example,  the ```MultibodyPositionToGeometryPose``` system takes in the joint positions of the robot and outputs the pose output required by the ```SceneGraph``` system.

## **Adding an end effector to the model**
[```example_iiwa_end_effector.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_iiwa_visualize.py)

The end effector model can be imported as a URDF/SDF file and be added to the multibody plant before finalizing and initializing it. 

In the example, we use a simple one finger defined in [```models/one_finger.urdf```](#) and adds it to out manipulation system. The [```example_iiwa_end_effector.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_iiwa_visualize.py) shows this functionality
## **Forward Kinematics**

[```example_FK.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_FK.py) 

Given the joint angles of the robot, we can set the corresponding joint position of the multibodyplant and then estimate the position of the end effector in the world/with respect to any other frame. The ```EvalBodyPoseInWorld``` function can be used to evaluate the position of the body in the world. Dake solves the kinematics automatically in the background. The example is available in [```example_FK.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_FK.py) 
## **Inverse Kinematics**
[```example_IK.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_IK.py) 

Drake has a rich [Inverse kinematics](https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_inverse_kinematics.html) library, based on numerical optimization. It also allows for addition of various types of constraints

##  **Estimating Cartesian Velocities**
[```example_velocity_estimate.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_velocity_estimate.py)

The cartesian velocities are estimated using the ```CalcJacobianSpatialVelocity``` method. It is then multiplied with the joint velocities to get the end effector velocity

## **Cartesian velocity control**
[```example_velocity_control.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_velocity_control.py)

A ```PseudoInverseVelocityController``` is implemented, which calculates the joint velocities from the desired end effector velocities and commands them

## **Estimating Cartesian forces**
[```example_force_estimate.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_force_estimate.py)

IIWA comes with joint torque sensors at all the 7 joints. This example uses jacobian transpose to estimate the forces and wrenches in cartesian space at the end effector from the measured joint torques. The accuracy of the system is approximateely 5 N 
## **Hybrid Force-Position control**
[```example_force_feedforward.py```](https://github.com/achuwilson/pydrake_iiwa/blob/main/example_force_feedforward.py)

This example makes use of the jacobian transpose pseudo inverse to calculate the feed forward torque for each joint of the IIWA. It can then be controlled along with the position control, resulting in a hybrid force-position control system
## **Motion Planning and Generating Trajectories** 
TODO