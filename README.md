# RaspTank2 con ROS 2

Adeept RaspTank V4 (Raspberry Pi 4) con **ROS 2 Humble** para clase: nodos
del robot, simulador ligero con RViz, rutas por puntos, visión (cubo rojo,
meta ArUco o un modelo YOLO entrenado) y la misión **coger el cubo y llevarlo
a la meta**. Los alumnos programan con una API sencilla en Python, igual en
simulación que en el robot real.

```bash
./scripts/build_ros2.sh                          # compilar (Ubuntu 22.04 + ROS 2 Humble)
./scripts/sim.sh mision mission:=true autostart:=true
ros2 run rasptank_alumno ej01_avanzar            # en otro terminal (source scripts/ros_env.sh)
```

Aplicación del PC (estado, parada de emergencia, mover, **calibración de la
odometría** manual o automática, simulación): `./scripts/app.sh` —
[tools/app/README.md](tools/app/README.md).

Guía completa: **[ROS2.md](ROS2.md)**. Robot real: [docker/README.md](docker/README.md).
Entrenar un modelo: [tools/yolo/README.md](tools/yolo/README.md).

---

# Adeept RaspTank-V4 Smart Car Kit for Raspberry Pi
Adeept RaspTank is an open source intelligent robotics product for artificial intelligence, robotics enthusiasts and students. This product is based on the Raspberry Pi motherboard using the python language and is compatible with the following Raspberry Pi models: 3B,3B+,4,5, etc.

## Resources Links

[RobotName]: Adeept RaspTank-V4 \
[Item Code]: ADR013-V4 \
[Official Raspberry Pi website]: https://www.raspberrypi.org/downloads/    \
[Official website]:  https://www.adeept.com/     \
[GitHub]: https://github.com/adeept/adeept_rasptank2/     


## Getting Support or Providing Advice

Adeept provides free and responsive product and technical support, including but not limited to:   
* Product quality issues 
* Product use and build issues
* Questions regarding the technology employed in our products for learning and education
* Your input and opinions are always welcome

We also encourage your ideas and suggestions for new products and product improvements
For any of the above, you may send us an email to:     \
Technical support: support@adeept.com      \
Customer Service: service@adeept.com


## About Adeept

Adeept was founded in 2015 and is a company dedicated to open source hardware and STEM education services. The Adeept technical team continuously develops new technologies, uses excellent products as technology and service carriers, and provides comprehensive tutorials and after-sales technical support to help users combine learning with entertainment. The main products include various learning kits and robots for Arduino, Raspberry Pi, ESP32 and BBC micro:bit.    \
Adeept is committed to assist customers in their education of robotics, programming and electronic circuits so that they may transform their creative ideas into prototypes and new and innovative products. To this end, our services include but are not limited to:   
* Educational and Entertaining Project Kits for Robots, Smart Cars and Drones
* Educational Kits to Learn Robotic Software Systems for Arduino, Raspberry Pi and micro: bit
* Electronic Component Assortments, Electronic Modules and Specialized Tools
* Product Development and Customization Services


## Copyright

Adeept brand and logo are copyright of Shenzhen Adeept Technology Co., Ltd. and cannot be used without written permission.
