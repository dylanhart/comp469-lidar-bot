import numpy as np

class PiBorgBot:
    def __init__(self, PBR):
        self.PBR = PBR

    def drive(self, left, right):
        self.PBR.SetMotor1(left)
        self.PBR.SetMotor2(-right)


class LidarBot(PiBorgBot):
    def __init__(self, PBR, ai, map, config):
        super().__init__(PBR)
        self.ai = ai
        self.map = map
        self.position = config['START_POS']
        self.dir = config['START_DIR']
        self.lidar = __import__(config['LIDAR_MODULE']).Lidar()

    def update(self):
        decision = self.ai.decide(self, self.lidar.get_image(), self.map)
        print(repr(decision))

        speed = decision['speed']
        angle = decision['angle']
        speed = np.clip(speed, -1, 1)
        angle = np.clip(angle, -1, 1)

        left, right = arcade(speed, angle)

        left = np.clip(left, -1, 1)
        right = np.clip(right, -1, 1)
        self.drive(left, right)
        print('left: {}, right: {}'.format(left, right))

    def stop(self):
        self.lidar.quit = True

def arcade(speed, angle):
    # http://robotpy.readthedocs.io/en/latest/wpilib/RobotDrive.html#wpilib.robotdrive.RobotDrive.arcadeDrive
    if speed > 0:
        if angle > 0.0:
            left = speed - angle
            right = max(speed, angle)
        else:
            left = max(speed, -angle)
            right = speed + angle
    else:
        if angle > 0.0:
            left = -max(-speed, angle)
            right = speed + angle
        else:
            left = speed - angle
            right = -max(-speed, -angle)
