import pycram.process_modules
# from .specialized_designators import *

from .datastructures.world import World
import signal
__version__ = "0.0.2"


def signal_handler(sig, frame):
    if World.current_world:
        World.current_world.exit()
    print("Exiting...")
    exit(0)

signal.signal(signal.SIGINT, signal_handler)



