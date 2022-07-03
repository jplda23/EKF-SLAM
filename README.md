"# LabSAut" 
 
The code in this repository was made for the course Autonomous Systems in Instituto Superior Tecnico.
It was made on top of the simulation developed by ISR (Instituto Superior de Robotica) and can be downloaded in this link.

The Ubunto version used was 16.04 and ROS Kinetic. Follow the steps to install both using this links:
LINK FOR UBUNTO 16.04 ---> https://releases.ubuntu.com/16.04/
LINK FOR ROS KINETIC ---> http://wiki.ros.org/kinetic

The code we developed is a SLAM algorithm based on Extended Kalman Filter (EKF) and uses Arucu Markers for the mapping algorithm. The code was validated by comparison using the gmapping algorithm as the standard. The code is also accompained by a paper.

Abstract: This paper investigates the convergence properties and consistency of an Extended Kalman Filter (EKF) based Simultaneous Localization and Mapping (SLAM) algorithm on a simulated robot and environment. Proofs of convergence are provided for the nonlinear two-dimensional SLAM problem with ArUco landmarks observed using a range-bearing sensor.

This work was graded 17/20 and has the following flaws:

Only presented results of individual runs that do not allow to evaluate the variability and robustness of the methods. At this point, the work could have explored more with the microsimulator, which allows the quantification of absolute errors and many experiments.

Explanations about the sensor problems as a justification for the less good results of the Gazebo simulator may not be true. In fact, the work does not show concrete examples of these failures, it was presented as a believe that this is what happens because this work had bad results. However, the reasons could be different. In particular, it doesn't mention how the alignment between the real map and the SLAM map was done and this is a serious problem.
The report has several inaccuracies and incomplete parts. It doesn't talk about the issue of alignment, state initialization and covariances, the outlier rejection algorithm is described in a superficial way, it is not possible to perceive if it is robust (for example, what happens if two landmarks change their identity from one image to the next - should also have used data association). Regarding inaccuracies:
 show as h the inverse observation model, instead of the direct observation model - in fact, this leaves the doubt if z is the (distance, angle) relative to the robot, or if the landmark position obtained from the inverse observation model.
It is not clear why convert the position of the marker, which is already in Cartesian coordinates, to polar coordinates. This forces to do more math and makes the Jacobian more complicated, in addition to the division by zero problem in the arctan of eq 8.
