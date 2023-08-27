import ctypes
from typing import Callable
ringcon = ctypes.CDLL("./ringcon_driver.dll")
ringcon_init:Callable = ringcon.ringcon_init
poll_ringcon:Callable = ringcon.poll_ringcon
if __name__ == "__main__":
    ringcon_init()
    while True:
        poll_ringcon()