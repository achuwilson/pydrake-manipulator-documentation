# pydrake-manipulator-docs

This document serves as a quick introduction to Kuka IIWA Robot and controlling it (or any other manipulator) using Drake Python API

## **Contents**
- [Introduction](#introduction)
- [Kuka System Architecture](#kuka-system-architecture)
- [Programming the Robot](#programming-the-robot)
    - [Kuka Sunrise WorkBench](#kuka-sunrise-workbench)
    - [FRI](#fri)
- Controlling from ROS    
- Drake Concepts    
- Drake drivers
    - Java App for Sunrise
    - kuka_driver LCM
-  LCM interface
 - Drake Intro
- Robot hardware Interface/Drake-LCM Interface
    - Loading a custom robot model
    - Reading from hardware
    - writing to hardware
- Joint Control
- Visualizing the robot state in Drake visualizers
- Adding an end effector to the model
- Forward Kinematics
- Inverse Kinematics
- Estimating Cartesian Velocities
- Cartesian velocity control
- Estimating Cartesian forces
- Hybrid Force-Position control
- Motion Planning -  Generating Trajectories 

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
  
  FRI stands for "Fast Research Interface", which is an addon provided by Kuka, which enables real time control of the robot system at the lowest level possible. This requires control signals be generated in an external computer and sent over a specific Ethernet port called KONI -  Kuka Optional Network Interface. The FRI is not enabled out of the box and has to be installed and enables through the Sunrise workbench. The default IP address of the FRI interface is ```192.170.10.2```

