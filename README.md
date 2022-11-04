# Inverted-Pendulum
<!-- ABOUT THE PROJECT -->
## About The Project
<p align="center"> 
<img src="https://user-images.githubusercontent.com/39928082/200025474-79a3ae22-2bb3-4f46-a87f-7ddd4ec8db48.gif" alt="SDU" title="SDU" width="80%" height="80%"/> 
</p>

### Abstract
In this project, we have worked to get a pendulum to balance using control theory. The pendulum itself is
mounted on a cart. The cart is controlled by a motor which is driven by a motor drive. We have programmed
the movement of the motor through MachineExpert, on a Schneider PLC. To configure the drive we have used
SoMove, where we have configured the signals between the drive and the PLC. To control the physical system,
we have set up equations of motion and created a model based on the physical model in Simulink. Here we have
been able to design controllers to comply with our performance requirements and simulated them in Simulink.
Once the model and controller has produced satisfying results, we transferred the controller to our physical
system. To verify the effect in the physical system, while driving the pendulum, we logged the angle of the
pendulum through a CFC program on the PLC, which we were able to plot to get illustrations of the controllers’
performance.

### Built With

* [MatLab]()
* [Schneider PLC (However not present in this repo)]()
* [Pure manpower]()
