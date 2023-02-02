'''
    Open the urdf model with Gepetto viewer
'''

from checkModel import visualizeGepetto, visualizePyBullet, visualizeMeshcat
import pinocchio as pin
import numpy as np
import time

urdf = 'model/urdf/climber.urdf'

if __name__ == '__main__':
    T = 200
    VEL = 3e-3
    
#    viz = visualizeGepetto(urdf, root_joint=pin.JointModelPlanar())
#    viz = visualizeGepetto(urdf, root_joint=pin.JointModelFreeFlyer())
    viz = visualizeGepetto(urdf)
    model = viz.model
    q = pin.neutral(model)
    q[0] = 0.2
    q[4] = -0.3
    q[7] = -0.3
    print('Starting configuration q:', q)
    
    for i in range(0, model.nv):
        print("Moving joint", i, model.names[i+1])
        v = np.zeros(model.nv)
        v[i] = VEL
        for t in range(T):
            if(t==int(T/4)):
                v[i] *= -1
            if(t==int(3*T/4)):
                v[i] *= -1
            q = pin.integrate(model, q, v)
            viz.display(q)
            time.sleep(0.015)