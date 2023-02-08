"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr
import pmt


class blk(gr.sync_block):  # other base classes are basic_block, decim_block, interp_block
    """Embedded Python Block example - a simple multiply const"""

    ctr = 0
    last_sys = 0
    last_rf = 0

    def __init__(self,  sample_rate = 0):  # only default arguments here
        """arguments to this function show up as parameters in GRC"""
        gr.sync_block.__init__(
            self,
            name='Print Tags',   # will show up in GRC
            in_sig=[np.complex64],
            out_sig=[np.complex64]
       
        )
        print("init print tags")
                  
        # a callback is registered (properties work, too).
        self.sample_rate = sample_rate
        
        
    
    def work(self, input_items, output_items):
    
        output_items[0][:] = input_items[0] * 1
        
        #print("ts", len(input_items[0]))
        tags = self.get_tags_in_window(0, 0, len(input_items[0]))
        
        blk.ctr += 1

        if len(tags) > 0:
            print("ctr", blk.ctr, "sample_rate", self.sample_rate, "tags", len(tags), "input_items", len(input_items[0]), "\n")
            
            for tag in tags:
                key = pmt.to_python(tag.key) # convert from PMT to python string
                value = pmt.to_python(tag.value) # Note that the type(value) can be several things, it depends what PMT type it was
                print('key:', key)
                print('value:', value, type(value))
                
                
                if key == 'sys_timestamp':
                    delta = value - blk.last_sys
                    time = delta / 40000000
                    
                    print("delta: ", delta, "time: ", time)
                    blk.last_sys = value
                
                if key == 'rf_timestamp':
                    delta = value - blk.last_rf
                    time = delta / self.sample_rate
                    
                    print("delta: ", delta, "time: ", time)
                    blk.last_rf = value
                    
                    
                print(" ")
            
        
        return len(input_items[0])
