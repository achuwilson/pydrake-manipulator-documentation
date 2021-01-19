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
-  [LCM interface](#)
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

The ROSJava nodes running on the robot as a Sunrise RobotApplication sends data and receives commands from a ROS master running on the external PC. The [wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki) provides detailed instructions on getting ROS working

## **Drake Concepts**

[Drake](https://drake.mit.edu/)is a C++ toolbox which can be used to model dynamical systems, solve mathematical problems and include multibody kinematics and dynamics. It also has a Python interface which is used exclusively in this document.

Drake uses an a network of systems which are interconnected through input and output ports. 

## **Drake drivers**
The [Drake IIWA Driver](https://github.com/RobotLocomotion/drake-iiwa-driver) allows for the interfacing of IIWA with Drake. This requires mainly two steps
 - Creating the Java Application
 - Compiling the ```kuka_driver```
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
## **LCM interface**
TODO

## **Robot hardware Interface/Drake-LCM Interface**
    - Loading a custom robot model
    - Reading from hardware
    - writing to hardware

## **Joint Control**
TODO
## **Visualizing the robot state in Drake visualizers**
TODO
## **Adding an end effector to the model**
TODO
## **Forward Kinematics**
TODO
## **Inverse Kinematics**
TODO
##  **Estimating Cartesian Velocities**
TODO
## **Cartesian velocity control**
TODO
## **Estimating Cartesian forces**
TODO
## **Hybrid Force-Position control**
TODO
## **Motion Planning and Generating Trajectories** 
TODO