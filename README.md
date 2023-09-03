# water-sensors-code
Repository with Arduino code that runs on the Arduino Uno, receiving data from water temperature and pH sensors and then relaying this data via Mavlink to the Pixhawk.

For the purpose of accelerating the development of customizing the Mavlink communication between Pixhawk and Arduino, the temperature sensor data is mapped as RUDDER_ANGLE, and the pH sensor data is mapped as SAIL_ANGLE.
