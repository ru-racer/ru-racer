# ru-racer

The goal to build RURacer-2 was to create improve platform performance, such as more accurate wheel encoders and more advanced embedded processors. Compared to RURacer-1, the new platform can reach four times lower latency (less than $7ms$) and twenty times faster computation power. To achieve this, an hardware architecture was designed along with the custom fitting of wheel encoders to monitor individual wheel speed and easily accessible and flexible hardware, software, and interfaces. An upgraded IMU system was also added to the new version capable of estimating the vehicle's rotations in three dimensions in addition to 3D angular velocities and linear accelerations. This platform allows researchers to take advantage of the latest software and hardware platforms, such as the Robotic Operating System (ROS) and the NVIDIA Jetson TX2 system on module (SOM).

<object width="425" height="350">
  <param name="movie" value="https://m.youtube.com/channel/UCU4D78hpTbhvZvqcQ8awvzw" />
  <param name="wmode" value="transparent" />
  <embed src="https://m.youtube.com/channel/UCU4D78hpTbhvZvqcQ8awvzw"
         type="application/x-shockwave-flash"
         wmode="transparent" width="425" height="350" />
</object>

![image](https://user-images.githubusercontent.com/26307692/115647143-4eee7300-a2f1-11eb-8875-1eae0d6352c9.png)


**Hardware and software architecture**
The scaled vehicle chosen for this project is a similar 1/7th scale Traxxas XO-1. The XO-1 shown in Figure~\ref{fig:2-scaleVehicle} is highly capable of drifting as it features an optional rear-wheel drive system and can reach up to $100$ miles per hour (MPH). To monitor individual wheels' angular velocity, optical wheel encoders (EM1 from US Digital) were custom fit and assembled on each side of the axles.

We chose NVIDIA Jetson-TX2 to control the vehicle because of it's high computational power for real-time processing. TX2 is running the Linux kernel with an Ubuntu operating system created by NVIDIA called L4T. Also, TX2 features onboard WiFi, which we set up to connect to the router and the host computer for real-time communications. The host computer provides high computational power at the edge for resource-hungry algorithms such as motion planning or image processing.  The communication framework is built on top of ROS to receive and send data from different processing nodes.

A microcontroller controls the steering wheel's servo and the speed of the brushless motor. Also, optical encoders are connected to the microcontroller, communicating to TX2 by Universal Asynchronous Receiver/Transmitter (UART) using \textit{rosserial}. Using UART, we achieve a $100\:Hz$ frequency for real-time sensor data acquisition and actuator control. This allows running the NMPC motion controller on TX2 while the lower level programming part is on the microcontroller.

The IMU used for RURacer-2 is provided by Adafruit-BNO055, providing 9 DOF sensing information, that it has been difficult before. We used a 3.2 Mega Pixels (MP) FLIR Blackfly GigE camera at $30$ frames per second, resulting in up to $2.6 mm$ of accuracy to localize the vehicle. The camera is connected to the network, and the image processing node is running on the edge processing unit, which is a powerful desktop computer. The architecture used for RURacer-2 is shown in Figure~\ref{fig:2-softwareArchitecture}.
