import numpy as np

def lspb(q0, q1, t, v):

    t0 = t
    t = [*range(0,t,1)]
    tf = max(t)

    v = abs(v)*np.sign(q0-q1)
    print(np.sign(q1-q0))
    return t

print(lspb(0,2,50,0))