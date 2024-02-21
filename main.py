import numpy as np
import methods as man

h_s = 1
time = 1
h = 0.01
h_sl = 0.1
square = man.create_body(h_s)
bt = man.move_body(time, h, square)

man.plot_trajectory(square, bt)
sp = man.make_through_space()
vf = man.plot_streamline(time, sp)

