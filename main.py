import json
import os
import sys
import threading
import time

import picoborgrev3.PicoBorgRev as PiBorg
from driver import LidarBot

# internal config
WATCHDOG_TIMEOUT = 1

# setup lib
PBR = PiBorg.PicoBorgRev()
PBR.Init()

# validate board is found
if not PBR.foundChip:
    boards = PiBorg.ScanForPicoBorgReverse()
    if len(boards) == 0:
        print('No PicoBorg Reverse found, check you are attached :)')
    else:
        print('No PicoBorg Reverse at address %02X, but we did find boards:' % (PBR.i2cAddress))
        for board in boards:
            print('    %02X (%d)' % (board, board))
        print('If you need to change the I²C address change the setup line so it is correct, e.g.')
        print('PBR.i2cAddress = 0x%02X' % (boards[0]))
    sys.exit()

# emergency stop, needs hardware btn (currently a jumper)
PBR.ResetEpo()


# watchdog thread to kill motors if program goes unresponsive
class Watchdog(threading.Thread):
    def __init__(self):
        super(Watchdog, self).__init__()
        self.event = threading.Event()
        self.terminated = False
        self.start()
        self.timestamp = time.time()
        self.timed_out = True

    def feed(self):
        self.event.set()

    def stop(self):
        self.terminated = True
        self.join()

    def run(self):
        # This method runs in a separate thread
        while not self.terminated:
            # Wait for a network event to be flagged for up to one second
            if self.timed_out:
                if self.event.wait(WATCHDOG_TIMEOUT):
                    # Connection
                    print('Reconnected...')
                    self.timed_out = False
                    self.event.clear()
            else:
                if self.event.wait(WATCHDOG_TIMEOUT):
                    self.event.clear()
                else:
                    # Timed out
                    print('Timed out...')
                    self.timed_out = True
                    PBR.MotorsOff()


# read config
config_name = os.environ.get('BOT_CONFIG', 'data/config.json')
with open(config_name) as config_file:
    config = json.load(config_file)

map_name = os.path.join(os.path.dirname(config_name), config['MAP'])
with open(map_name) as map_file:
    map = json.load(map_file)

ai_name = config['AI']
ai_module = __import__(ai_name)
if 'AI_CONFIG' in config:
    ai = ai_module.AI(config['AI_CONFIG'])
else:
    ai = ai_module.AI()

# setup bot driver
bot = LidarBot(ai, map, config)
watchdog = Watchdog()

# drive!
watchdog.start()
try:
    while True:
        bot.update(PBR)

        if watchdog.timed_out:
            print('watchdog timed out, quitting')
            break
        else:
            watchdog.feed()
except KeyboardInterrupt:
    print('shutting down.')
finally:
    # cleanup
    watchdog.stop()

    # kill motors
    PBR.MotorsOff()

    print('exiting.')
