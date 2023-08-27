import idc
import idaapi
def nav_nxt():
    idaapi.next_head(idc.get_screen_ea())
def nav_back():
    idaapi.prev_head(idc.get_screen_ea())
def nav_step_in():
    inst = idaapi.insn_t()
    idaapi.decode_insn(inst, idc.get_screen_ea())