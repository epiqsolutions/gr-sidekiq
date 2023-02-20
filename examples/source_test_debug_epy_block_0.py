"""
Embedded Python Blocks:

Each time this file is saved, GRC will instantiate the first class it finds
to get ports and parameters of your block. The arguments to __init__  will
be the parameters. All of them are required to have default values!
"""

import numpy as np
from gnuradio import gr


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
        # a callback is registered (properties work, too).
        self.example_param = example_param

    def work(self, input_items, output_items):


        expected = input_items[i]

        #validate samples
        for j in len(input_items): 
            this_value = input_items[j]

            if (this_value != expected):
                print("bad value", j, "expected", expected, "value", this_value)

                """
                #error print the buffer around the error
                for k in range(-5, 5):
                    print((j + k), " ", rx_buff[i][j+k])
                """
                break

            expected = (this_value + 1)
            if expected == (max_data + 1):
                expected = -(max_data+1)

        return len(output_items[0])
