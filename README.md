# MotorDaemon (WIP)
A feedback loop motor deamon with odometer support for the BeagleBone Black

This daemon consist of multiple threads to handle odometry, motors, actuators and a piloting interface.
It has a TCP based protocol to let another app control it.
It can also be launched directly and piloted through stdin, this is the default behaviour at the moment.
This is still a WIP, so there's no documentation available at the moment.

"./MotorDaemon -s"  : Launches MotorDaemon in daemon mode (server listening on localhost at the moment)

This project is part of INTechOS (https://github.com/discord-intech/meta-intechos)
