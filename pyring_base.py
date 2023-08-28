import ctypes
from typing import Callable
import os
print("import dll...")
ringcon = ctypes.CDLL(f"{os.path.dirname(__file__)}/ringcon_driver.dll")
ringcon_init:Callable = ringcon.ringcon_init

class pool_ret_data(ctypes.Structure):
    _fields_=[("running",ctypes.c_bool),
              ("squatting",ctypes.c_bool),
              ("ringcon_pushval",ctypes.c_int)]

poll_ringcon:Callable = ringcon.poll_ringcon
