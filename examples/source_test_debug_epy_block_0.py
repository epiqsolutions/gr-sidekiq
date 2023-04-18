"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr
rx_resolution = 12
max_data = (1 << (rx_resolution -1))-1
print(max_data)

debug_ctr = 0
byte_num = 0


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    def __init__(self, example_param=1.0):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Embedded Python Block',   # will show up in GRC
            in_sig=[np.int16],
            out_sig=[np.int16]
        )
        # if an attribute with the same name as a parameter is found,
        # a callback is registered (properties work, too).j
        self.example_param = example_param

    def work(self, input_items, output_items):
        global debug_ctr, byte_num
        
        
        expected = input_items[0][0]

        #validate samples
        for j in range(len(input_items[0])): 
            this_value = input_items[0][j]

            if (this_value != expected):
                print("Fail: Byte # ", hex(byte_num + (j*2)), "expected", hex((0xffff & expected)), "actual", hex((0xffff & this_value)))                

            expected = (this_value + 1)
            if expected == (max_data + 1):
                expected = -(max_data+1)
                
                
        debug_ctr += 1
        if debug_ctr % 1000 == 0:
            print("ninput_items", len(input_items[0]))
            
        byte_num = byte_num + (len(input_items[0]) * 2)
        
        return len(output_items[0])
