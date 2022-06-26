"# LabSAut" 
 
The code in this repository was made for the course Autonomous Systems in Instituto Superior Tecnico.
It was made on top of the simulation developed by ISR (Instituto Superior de Robotica) and can be downloaded in this link.

The Ubunto version used was 16.04 and ROS Kinetic. Follow the steps to install both using this links:
LINK FOR UBUNTO 16.04 ---> https://releases.ubuntu.com/16.04/
LINK FOR ROS KINETIC ---> http://wiki.ros.org/kinetic

The code we developed is a SLAM algorithm based on Extended Kalman Filter (EKF) and uses Arucu Markers for the mapping algorithm. The code was validated by comparison using the gmapping algorithm as the standard. The code is also accompained by a paper.

Abstract: This paper investigates the convergence properties and consistency of an Extended Kalman Filter (EKF) based Simultaneous Localization and Mapping (SLAM) algorithm on a simulated robot and environment. Proofs of convergence are provided for the nonlinear two-dimensional SLAM problem with ArUco landmarks observed using a range-bearing sensor.