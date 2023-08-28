import keyboard_ctl
import idc
import idaapi
def move_instr_cur(direction:bool):
    cur = idc.get_screen_ea()
    if direction:
        idaapi.jumpto(idaapi.next_head(cur, cur + 15))
    else:
        idaapi.jumpto(idaapi.prev_head(cur, cur - 15))


'''
these are dirty hacks, but worked well at least for short operations
'''
def nav_step_in():
    VK_RETURN = 0x0D
    keyboard_ctl.trigger_key(VK_RETURN)

def nav_back():
    VK_ESCAPE = 0x1B
    keyboard_ctl.trigger_key(VK_ESCAPE)

def decompile_func():
    VK_F5 = 0x74
    keyboard_ctl.trigger_key(VK_F5)

# ref: https://github.com/pwndbg/pwndbg/issues/844#issuecomment-812961187
from xmlrpc.server import SimpleXMLRPCServer
import threading
mutex = threading.Condition()
def wrap(f):
    def wrapper(*a, **kw):
        rv = []
        error = []

        def work():
            try:
                result = f(*a, **kw)
                rv.append(result)
            except Exception as e:
                error.append(e)

        with mutex:
            flags = idaapi.MFF_WRITE
            idaapi.execute_sync(work, flags)

        if error:
            msg = 'Failed on calling {}.{} with args: {}, kwargs: {}\nException: {}' \
                .format(f.__module__, f.__name__, a, kw, str(error[0]))
            print('[!!!] ERROR:', msg)
            raise error[0]

        return rv[0]

    return wrapper

IDA_RPC_HOST = 'localhost'
IDA_RPC_PORT = 8787
server:SimpleXMLRPCServer = None
server_thread:threading.Thread = None
def create_ida_rpc_server():
    global server
    global server_thread
    server = SimpleXMLRPCServer((IDA_RPC_HOST, IDA_RPC_PORT), logRequests=True, allow_none=True)
    server.register_introspection_functions()
    
    server.register_function(wrap(move_instr_cur))
    server.register_function(wrap(nav_back))
    server.register_function(wrap(nav_step_in))
    server.register_function(wrap(decompile_func))
    

    print('IDA Pro xmlrpc hosted on http://%s:%s' % (IDA_RPC_HOST, IDA_RPC_PORT))
    print('Call `del_ida_rpc_server()` to shutdown the IDA Pro xmlrpc server.')
    # Run the server's main loop
    server_thread = threading.Thread(target=server.serve_forever)
    server_thread.daemon = True
    server_thread.start()

def del_ida_rpc_server():
    global server
    global server_thread
    
    server.shutdown()
    server.server_close()
    
    del server
    del server_thread
    
