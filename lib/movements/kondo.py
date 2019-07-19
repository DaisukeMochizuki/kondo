#coding: UTF-8
from Rcb4BaseLib import Rcb4BaseLib
import time

# Main constants of Kondo
ROTATION_ANGLE = 10
STEP_DISTANCE = None
DEADLOCK_TIME = 100


class Kondo:
    state_dict = {
        "stand": 1,
        "step_forward": 47,
        "step_backward": 48,
        "small_step_forward": 4,
        "small_step_backward": 5,
        "turn_right": 6,
        "turn_left": 45,
        "tilt": 8,
        "straighten": 9,
        "triple_jump": 12,
        "shoot": 6,
        "archery_stand": 3,
        "grab_arrow": 4,
        "stretch_bow": 5,
        "body_turn_right": 20,
        "body_turn_left": 21}

    def __init__(self, debug=False, device='/dev/ttySAC4'):
        self.rcb4 = Rcb4BaseLib()
        self.state = -1
        self.debug = debug
        self.init(device)

    def init(self, device):
        if not self.debug:
            try:
                if device != '/dev/ttySAC4':
                    print("trying to connect to " + device)
                    if self.rcb4.open(device, 115200, 1.3):
                        print("Connection to " + device + " established")
                        self.state = 0
                    else:
                        raise AttributeError

                else:
                    print("trying to connect via UART4")
                    if self.rcb4.open(device, 115200, 1.3):
                        print("Connection established via UART4")
                        self.state = 0
                    else:
                        print("trying to connect via USB")
                        if self.rcb4.open(device, 115200, 1.3):
                            print("Connection established via USB")
                            self.state = 0
                        else:
                            print("trying to connect via UART")
                            for i in range(5):
                                if self.rcb4.open('/dev/ttySAC' + str(i), 115200, 1.3):
                                    print("Connection to UART" + str(i) + "established")
                                    self.state = 0
                                    break
                if self.state == -1:
                    print("No connection found")
                    exit()

            except AttributeError:
                print("Check connection and enum lib")
                exit()

    def get_state(self):
        if self.state < 0:
            print("Do Kondo.init() function")
            return -1
        else:
            motion_num = self.rcb4.getMotionPlayNum()
            if motion_num < 0:
                print('motion get error', motion_num)
                return -1
        return motion_num

    def run_motion(self, motion):
        if not self.debug:
            if motion < 0:
                print('Motion error')
            else:
                self.rcb4.motionPlay(motion)
                deadlock_time = time.time()
                while True:
                    self.state = self.get_state()
                    if self.state <= 0:
                        break
                    time.sleep(0.1)
                    if deadlock_time - time.time() > DEADLOCK_TIME:
                        print("Deadlock")
                        break
            if self.state == 0:
                return 0
            else:
                return -1
        else:
            return 0

    def walk(self, step_num):
        if step_num > 0:
            for i in range(step_num):
                if self.run_motion(self.state_dict['step_forward']) == -1:
                    return -1
            return 0
        elif step_num < 0:
            for i in range(abs(step_num)):
                if self.run_motion(self.state_dict['step_backward']) == -1:
                    return -1
            return 0
        return 0

    def small_walk(self, step_num):
        if step_num > 0:
            for i in range(step_num):
                if self.run_motion(self.state_dict['step_forward']) == -1:
                    return -1
            return 0
        elif step_num < 0:
            for i in range(abs(step_num)):
                if self.run_motion(self.state_dict['step_backward']) == -1:
                    return -1
            return 0
        return 0

    def turn(self, angle):
        if angle > 0:
            for i in range(angle // ROTATION_ANGLE):
                if self.run_motion(self.state_dict['turn_right']) == -1:
                    return -1
            return 0
        elif angle < 0:
            for i in range(abs(angle) // ROTATION_ANGLE):
                if self.run_motion(self.state_dict['turn_left']) == -1:
                    return -1
            return 0
        return 0

    def tilt(self):
        if self.run_motion(self.state_dict['tilt']) == -1:
            return -1
        else:
            return 0

    def straighten(self):
        if self.run_motion(self.state_dict['straighten']) == -1:
            return -1
        else:
            return 0

    def triple_jump(self):
        if self.run_motion(self.state_dict['triple_jump']) == -1:
            return -1
        else:
            return 0

    def body_turn(self, angle):
        if angle > 0:
            for i in range(angle):
                if self.run_motion(self.state_dict['body_turn_right']) == -1:
                    return -1
            return 0
        elif angle < 0:
            for i in range(abs(angle)):
                if self.run_motion(self.state_dict['body_turn_left']) == -1:
                    return -1
            return 0
        return 0

    def archery_stand(self):
        if self.run_motion(self.state_dict['archery_stand']) == -1:
            return -1
        else:
            return 0

    def grab_arrow(self):
        if self.run_motion(self.state_dict['grab_arrow']) == -1:
            return -1
        else:
            return 0

    def stretch_bow(self):
        if self.run_motion(self.state_dict['stretch_bow']) == -1:
            return -1
        else:
            return 0

    def shoot(self):
        if self.run_motion(self.state_dict['shoot']) == -1:
            return -1
        else:
            return 0

    def __del__(self):
        print("Closing connection")
        self.rcb4.close()
        self.state = -1


if __name__ == "__main__":
   kondo = Kondo()
   kondo.init()
   kondo.walk(4)
