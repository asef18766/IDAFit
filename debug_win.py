from tkinter import *
from pyring_base import pool_ret_data
from typing import Dict, List
frame = Tk()
frame.geometry("480x360")
display_fields:List[str] = [ f[0] for f in pool_ret_data._fields_ ]

def label_ctor()->Label:
    display = Label(frame, text="", justify="left")
    display.pack(anchor="w", pady=2, side= TOP)
    #display.grid(row=r_idx, column=1)
    return display

display_fmap:Dict[str, Label] = { k:label_ctor() for k in display_fields }
display_fmap.update({"others":label_ctor()})

def update_win(data:pool_ret_data, others=None):
    for f in display_fields:
        display_fmap[f].configure(text=f"{f}: {str(getattr(data, f))}")
    if others != None:
        display_fmap["others"].configure(text=f"others: {str(others)}")
    frame.update()