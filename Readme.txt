Application Name:
MMITSS FIELD

Version Number:
v1.0

Installation Instructions:
There is no installer included with this package. Only Source code is provided

Description:
There are four major component of MMITSS field version: Intelligent Signal Control (I-SIG), Signal Priority (SP), 
Mobile Accessible Pedestrian Signal System (PED-SIG) and real-time performance observer (PERF-OBS).

Intelligent Signal Control (I-SIG)
I-SIG provides real-time adaptive signal control to general vehicles by allocating green time of each phase intelligently. 
I-SIG takes vehicle trajectory data from BSMs and solve for optimal signal schedule in terms total delay minimization or 
total queue length minimization. The optimal signal schedule are implemented to Signal Controllers through NTCIP commands.

Signal Priority (SP)
Signal priority provides priority to different modes of vehicles including transit, trucks and emergency vehicles. Priority
 eligible vehicles receive SPAT, MAP and SSM from RSE when they enter the DSRC range and generate SRMs. RSE considers all 
the active requests from different vehicle and solve for optimal signal schedule in terms of minimization of priority 
vehicle delay. The optimal signal schedule are implemented to Signal Controllers through NTCIP commands.

Mobile Accessible Pedestrian Signal System (PED-SIG)
PED-SIG includes a smartphone application and two RSE applications that can send pedestrian signal request and provide 
assistance to disabled pedestrians. The RSE broadcast Pedestrians map information through WIFI and the smartphone application 
receives the MAP and sends pedestrian signal request. RSE will put pedestrian signal request to the Signal Controller.

Real-time Performance Observer (PERF-OBS)
PERF-OBS monitors all the users on the road through BSMs and collects real-time performance data by movement by mode such as 
travel time, delay, and queue length etc. 


The field version of MMITSS should be running on Savari STREETWAVE (RSE) or Savari MobiWAVE (ASD).

The source code and libraries are in the following folders:
Android: An Android phone application for pedestrian application
Configuration Files: Sample configuration files for all MMITSS applications
docs: a user guide which contains a short introduction of MMITSS and how to run each MMITSS application
OBE: Source code of applications on OBE
RSE: Source code of applications on RSE
Library: GLPK library
