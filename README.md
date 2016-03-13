# CreepyFirmware
Creepyfirmware is the code running Creepi, a little creepy robotic arm with a webcam tracking people's faces.

Creepi is a little, useless and creepy robotic arm with a webcam at its end. It basically stares at people and tracks them.

The arm is entirely composed of 3D printed parts, all designed and printed by Damien Delbos. It uses Dynamixel AX-12a servomotors and a regular Logitech webcam. Ultimately, it is supposed to run on an Intel Edison but for now runs on a regular computer.

The algorithm uses OpenCV and is fairly simple. It uses face detection, video tracking and PID loops.

Technologies used include:

- 3D printing
- Autodesk Inventor
- C++
- OpenCV 3.0
- Dynamixel AX-12a
- Intel Edison
- GnuPlot (for debugging)

Demo available at https://www.youtube.com/watch?v=oSjFuTQfOhY
