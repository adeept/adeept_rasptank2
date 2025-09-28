class Movement:

    def __init__(self, controller, motor1_direction, motor2_direction):
        self.__motor1 = controller.motor(1)
        self.__motor1_direction = motor1_direction
        self.__motor2 = controller.motor(2)
        self.__motor2_direction = motor2_direction
        self.__speed = 100

    def forward(self):
        self.__motor1.throttle = self.__motor1_direction * self.__speed
        self.__motor2.throttle = self.__motor2_direction * self.__speed

    def backward(self):
        self.__motor1.throttle = -self.__motor1_direction * self.__speed
        self.__motor2.throttle = -self.__motor2_direction * self.__speed

    def left(self, angle=90):
        self.__motor1.throttle = self.__motor1_direction * self.__speed
        self.__motor2.throttle = -self.__motor2_direction * self.__speed

    def right(self):
        self.__motor1.throttle = -self.__motor1_direction * self.__speed
        self.__motor2.throttle = self.__motor2_direction * self.__speed

    def stop(self):
        self.__motor1.throttle = 0
        self.__motor2.throttle = 0

    def speed(self, speed):
        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0
        self.__speed = speed