import numpy as np
from math import sqrt

class AI:
    def __init__(self):
        pass

    def decide(self, bot, image, map):
        """
        Takes a list of distances to nearby obstacles and computes move parameters.
        :param image: np.array of distances from surrounding obstacles as measured by a 360-degree LIDAR
        :returns angle: represents the clockwise angle (radians) to turn at that time
        :returns speed: scales the bot's velocity
        :returns message: message to show as a label by the bot
        """
        message = ''
        angle = 0

        # choose left or right, whatever has most distant obstacles
        mid = len(image) // 2
        l = image[:mid]  # left side of the bot
        r = image[mid:]  # right side of the bot
        fl = image[mid // 2:mid]  # front left quarter
        fr = image[mid:mid + mid // 2]  # front right quarter
        bl = image[:mid // 2]  # back left quarter
        br = image[mid + mid // 2:]  # back right quarter

        # evaluate the "volume" of obstacles on the left vs. right and select the direction minimizing the
        # the chance for a collision
        # if min(fl) * np.average(fl) ** 0.5 > min(fr) * np.average(fr) ** 0.5:
        if sqrt(min(fl) * np.average(fl)) > sqrt(min(fr) * np.average(fr)):
            angle = -1
            message += 'left'
        else:
            angle = 1
            message += 'right'

        # determine "safe" speed
        # first, calculate the "front" quarter of the LIDAR image
        front = image[3 * mid // 4: 5 * mid // 4]
        # consider the closest obstacle in the front as the guideline for the speed
        speed = pow(min(front) / np.average(image), 2)
        message += '\nspeed:{:2.0f}'.format(speed * 100)

        return {'angle': angle, 'speed': speed, 'quote': message}

