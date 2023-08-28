import os
import sys
import subprocess

def start_event_loop():
    from pyring_base import pool_ret_data, ringcon_init, poll_ringcon
    from debug_win import update_win
    import ctypes
    ringcon_init()
    pret = pool_ret_data()
    while True:
        poll_ringcon(ctypes.pointer(pret))
        update_win(pret)


# note: directly ida operation on non-main thread or dll loading will crash ida, shall create a process instead
'''
create a process to handle ringcon operations, 
and pass back to ida daemon thread to execute action
'''
def create_ringcon():
    py_path = os.path.join(sys.exec_prefix, 'pythonw.exe')
    DETACHED_PROCESS = 8
    dir_name = os.path.dirname(__file__)
    subprocess.Popen([py_path, "-c", 'import pyring;pyring.start_event_loop()'],
                     creationflags=DETACHED_PROCESS,
                     stdout=open(dir_name + "/stdout.txt", "w"),
                     stderr=open(dir_name + "/stderr.txt", "w"),
                     cwd=dir_name)

if __name__ == '__main__':
    from ida_tools import create_ida_rpc_server
    # start ida daemon
    create_ida_rpc_server()

    # ringcon daemon creation
    create_ringcon()
    