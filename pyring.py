import os
import sys
import subprocess
from datetime import datetime
from typing import Callable, Dict


# hyper parameters
SQUATTING_TIME = 2
SQUATTING_THRESHOLD = 0.5

class SQUATTING_STATES:
    NOT = 0
    DOING = 1
    DONE = 2
SQUATTING_STATE = SQUATTING_STATES.NOT
SQUATTING_TS = -1
SQUATTING_CTR = 0
SQUATTING_EV_CBACK:Dict[int, Callable[[], None]] = {}
SQUATTING_EV = [0, 0] # (pass cnt, cnt)

OPERATION_PROXY = None
SQUATTING_EV_CBACK.update({3:(lambda:OPERATION_PROXY.decompile_func())})

def squatting_detector(data):
    global SQUATTING_STATE
    global SQUATTING_TS
    global SQUATTING_CTR
    global SQUATTING_EV
    if SQUATTING_STATE == SQUATTING_STATES.DOING:
        cur_ts = datetime.now().timestamp()
        if data.squatting:
            SQUATTING_EV[0] += 1
            SQUATTING_EV[1] += 1
            if cur_ts - SQUATTING_TS >= SQUATTING_TIME and SQUATTING_EV[0] / SQUATTING_EV[1] >= SQUATTING_THRESHOLD:
                print("pass squatting!!")
                SQUATTING_CTR += 1
                SQUATTING_STATE = SQUATTING_STATES.DONE
                SQUATTING_TS = -1
                SQUATTING_EV = [0, 0]
        else:
            if cur_ts - SQUATTING_TS >= SQUATTING_TIME: # not within the timestamp
                print("not standard enough!!")
                SQUATTING_TS = -1
                SQUATTING_STATE = SQUATTING_STATES.NOT
                SQUATTING_EV = [0, 0]
            else:
                SQUATTING_EV[1] += 1
                
    elif SQUATTING_STATE == SQUATTING_STATES.NOT:
        if data.squatting:
            print("start squatting!!")
            SQUATTING_TS = datetime.now().timestamp()
            SQUATTING_STATE = SQUATTING_STATES.DOING
    elif SQUATTING_STATE == SQUATTING_STATES.DONE:
        if not data.squatting:
            print("reset squatting!!")
            if SQUATTING_CTR in SQUATTING_EV_CBACK: # trigger when release
                SQUATTING_EV_CBACK[SQUATTING_CTR]()
                SQUATTING_CTR = 0
            SQUATTING_STATE = SQUATTING_STATES.NOT
            

PUSHING_THRESHOLD = 19
PULLING_THRESHOLD = 8
MOV_THRESHHOLD = 0.5
RUN_TS = -1

def ida_move_detector(data):
    global RUN_TS
    if data.running:
        cur = datetime.now().timestamp()
        if RUN_TS == -1:
            RUN_TS = cur
        elif cur - RUN_TS >= MOV_THRESHHOLD:
            RUN_TS = -1
            if data.ringcon_pushval >= PUSHING_THRESHOLD:
                print("activate pushing")
                OPERATION_PROXY.move_instr_cur(True)
            elif data.ringcon_pushval <= PULLING_THRESHOLD:
                print("activate pulling")
                OPERATION_PROXY.move_instr_cur(False)
    else:
        RUN_TS = -1


def start_event_loop():
    from pyring_base import pool_ret_data, ringcon_init, poll_ringcon
    from debug_win import update_win
    import ctypes
    import xmlrpc.client
    global OPERATION_PROXY
    ringcon_init()
    OPERATION_PROXY = xmlrpc.client.ServerProxy('http://localhost:8787')
    print(OPERATION_PROXY.system.listMethods())
    pret = pool_ret_data()
    while True:
        poll_ringcon(ctypes.pointer(pret))
        squatting_detector(pret)
        ida_move_detector(pret)
        update_win(pret, SQUATTING_CTR)


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
    