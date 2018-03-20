# arduino_IR_linefollower

Code for Arduino project in which a car follows a line on the floor through an 8-array InfraRed Sensor and different control modes (PID, PI, PD, P,...).
Notes: 
In my project I had to use a 9V battery due to resources avaible, but it is highly recommended to used a LiPo battery instead, because servo motors drain the battery incredibly fast.
Also, due to lack of resources and material, I had to make my own driver for the servos, but you can easily purchase it online. The mission of the driver is to give the servos enough power to move, because the output current of Arduino digital pins is limited and not enough to move them.
Parameters for control loops are to be chosen experimentally (or you could implement an algorithm to optimize them).

It's a really fun project and a perfect opportunity to enter the Arduino world with a step by step guide.

Hope you enjoy it!
