Velociryder Code
================

This is the software for the Velociryder, a self balancing two wheeled 
skateboard, started as a side project and was then entered in Georgia Tech's
2011 InVenture Prize. 

Features
--------

The code has the following features:
* PD control (no integral, wasn't necessary)
* Automatic startup via killswitch and level detection
* Support for foot pedal steering, just as in the Velociryder
* Use of accelerometer and gyroscope to obtain board level
* Moving average and complementary filters for the sensors

It works alright. 

Usage
-----
Things you will probably need to tweak:
### In velociryder.pde
* Turning constants

### In sensors.pde
* GYROTORAD4, GYROTORAD, ACCLTORAD: These convert the sensor's ADC units to 
	radians
* ACCL_CENTER: This defines the center balance point of the board
* sensor defines: The pins the sensors are on
* OHSHITSWITCH: killswitch pin

This is Arduino source code. So put it in a directory called velociryder in 
your sketchbook, fiddle, program, and go!

Implementations
---------------
* Version 1 of the Velociryder, back when it was called the Glide Board
http://www.youtube.com/watch?v=xvfUIxusPZw

* Version 2, built in aluminum and eliminated hand controller
http://www.gpb.org/inventure/velociryder

License
-------
This code is released under the MIT License. If you do use it, send me a 
message, I want to see it!
