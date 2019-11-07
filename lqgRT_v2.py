import matlab.engine
import numpy as np

eng = matlab.engine.start_matlab()
# t = eng.linspace(-5, 0, 1e4)
# x1 = t
# x2 = 5*eng.ones(eng.size(t))
# tf = eng.lqgRT_v2(1, 1e4, 2, 2, 2, matlab.int8([2]), eng.eye(2), eng.eye(2), [1 1; 1 -1], eng.eye(2), eng.eye(2), eng.eye(2), 1e-3*eng.eye(2), matlab.int8([-5; 5]), 1, 1, matlab.int8([0; 5]), matlab.int8([0; 2.5]), t, matlab.int8([x1; x2]), 5);
eng.lqgRT_v3(nargout=0)
