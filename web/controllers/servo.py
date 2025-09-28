from __future__ import division
import time
import threading

CLOCKWISE = 1
ANTICLOCKWISE = -1

class ServoCtrlThread(threading.Thread):
    
    def __init__(self, name, controller, channel_number, position, direction):

        self.__name = name
        self.__servo = controller.servo(channel_number)

        # Constants
        self.servo_direction = direction
        self.angle_maximum_range = 180
        self.angle_minimum_range = 0

        # Variables
        self.servo_speed = 1
        self.angle_initial_value = position
        self.angle_target_value = position
        self.angle_current_value = position
        self.move_direction = direction

        self.move_step_size = 1
        self.move_step_duration = 0.09
        self.move_step_delay = 0 # No delay between steps

        self._running = True
        super().__init__()
        self.__flag = threading.Event()
        self.__flag.clear()
        self.start()

        self.__set_angle(self.angle_initial_value)

    ##################################
    ########## SERVO Action ##########
    ##################################            
    def clockwise(self):
        self.move(direction=CLOCKWISE)
        
    def anticlockwise(self):
        self.move(direction=ANTICLOCKWISE)

    def stop(self):
        self.pause()
        self.angle_target_value = int(round(self.angle_current_value, 0))
        self.__set_angle(self.angle_target_value)

    def stop_thread(self, timeout=1.0):
        """Signals the thread to stop and waits for it to terminate."""
        self._running = False
        # Wake the thread if it is waiting on the flag
        self.__flag.set()
        self.join(timeout)

    def move_to(self, angle):
        self.move(angle=angle)

    def move(self, *, direction=CLOCKWISE, speed = 1, number_of_steps=None, angle=None):
        self.move_direction = direction * self.servo_direction
        self.servo_speed = speed
        if angle is not None:
            self.angle_target_value = angle
            if angle > self.angle_current_value:
                self.move_direction = CLOCKWISE
            else:
                self.move_direction = ANTICLOCKWISE
        elif number_of_steps is None:
            if self.move_direction == CLOCKWISE:
                self.angle_target_value = self.angle_maximum_range
            else:
                self.angle_target_value = self.angle_minimum_range
        else:
            self.angle_target_value = int(round(self.angle_current_value + self.move_direction * number_of_steps * self.move_step_size, 0))
        #print(f'-> {self.__name} move to {self.angle_target_value} from {self.angle_current_value} at speed {self.servo_speed} direction {self.move_direction}')
        self.resume()

    def __next_step(self):
        new_position = self.angle_current_value + (self.move_direction * self.servo_speed * self.move_step_duration)
        #print(f'{time.time()} -> start next step for -> {new_position}')
        self.__set_angle(new_position)
        time.sleep(self.move_step_delay)
        #print(f'{time.time()} -> finish next step for -> {new_position}')
        
    ##################################
    ########## SERVO PWM    ##########
    ##################################
    def incrementPwm(self):
        self.angle_target_value = self.angle_current_value + 1
        self.__set_angle(self.angle_target_value)
        
    def derementPwm(self):
        self.angle_target_value = self.angle_current_value - 1
        self.__set_angle(self.angle_target_value)

    def reset(self):
        self.pause()
        self.angle_target_value = self.angle_initial_value
        self.__set_angle(self.angle_initial_value)
        
    def __set_angle(self, angle):
        self.angle_current_value = self.__sanitize_angle(angle)
        #print(f'-> {self.__name} set angle to {self.angle_current_value} (wanted {angle})')
        self.__servo.angle = int(round(self.angle_current_value, 0))

    def __sanitize_angle(self, angle) -> float:
        if self.move_direction == CLOCKWISE and angle > self.angle_target_value \
            or self.move_direction == ANTICLOCKWISE and angle < self.angle_target_value:
            #print(f'{time.time()} -> {self.__name} pause as angle reached Target {angle}')
            self.pause()
            return self.angle_target_value
        elif angle > self.angle_maximum_range:
            #print(f'{time.time()} -> {self.__name} pause as angle reached Max Angle {angle}')
            self.pause()
            self.angle_target_value = self.angle_maximum_range
            return self.angle_maximum_range
        elif angle < self.angle_minimum_range:
            #print(f'{time.time()} -> {self.__name} pause as angle reached Min Angle {angle}')
            self.pause()
            self.angle_target_value = self.angle_minimum_range
            return self.angle_minimum_range
        return angle

    ##################################
    ####### Thread Management ########
    ##################################
    def run(self):
        while self._running:
            self.__flag.wait()
            self.__next_step()
            pass
            
    def pause(self):
        print(f'{time.time()} -> pause {self.__name}')
        self.__flag.clear()

    def resume(self):
        print(f'{time.time()} -> resume {self.__name}')
        self.__flag.set()
   

