'''
'''

from checkModel import visualizeGepetto, visualizePyBullet, visualizeMeshcat
import pinocchio as pin
import numpy as np
from numpy.linalg import norm
import time
np.set_printoptions(precision=3, linewidth=200, suppress=True)

urdf = 'model/urdf/climber.urdf'

class ClimberSimulator:

    # Class constructor
    def __init__(self, urdf):
        self.use_viewer = True
        self.viewer = visualizeGepetto(urdf)
        self.model = self.viewer.model
        self.gui = self.viewer.viewer.gui
        self.WIRE_RADIUS = 0.005
        
        # The "line" was not showing well in the viewer so we don't use it anymore
#        self.line_name = "world/pinocchio/wire"
#        self.gui.addLine(self.line_name, [0.,0.,0.], [0.,0.,-5], [1.,1.,1.,1.])
#        self.gui.setLineStartPoint(self.line_name, [0.,0.,0.])
        
        # hide the wire visualization from the urdf and replace it with a variable length arrow in the viewer
        self.gui.setScale("world/pinocchio/visuals/wire_0", [.0, .0, .0])
        self.WIRE_NODE_NAME = "world/pinocchio/wire"
        self.gui.addArrow(self.WIRE_NODE_NAME, self.WIRE_RADIUS, 1.0, (1., 0., 0., 1.)) # name, ray, length, color
        
        # add a floor to represent the wall
        self.FLOOR_NAME = "world/pinocchio/visuals/mountain_floor"
        self.gui.addFloor(self.FLOOR_NAME)
        self.gui.setLightingMode(self.FLOOR_NAME, "OFF")
        self.gui.applyConfiguration(self.FLOOR_NAME, 
                                    pin.SE3ToXYZQUAT(pin.SE3(pin.utils.rotate('y', np.pi/2), np.zeros(3))).tolist())
        
        
        q = pin.neutral(self.model)
        q[0] = 0.2
        q[4] = -0.3
        q[7] = -0.3
        
        self.WIRE_BASE_JOINT_ID = self.model.getJointId("wire_base_pitch")
        self.WIRE_PRISMATIC_JOINT_ID = self.model.getJointId("wire_base_prismatic")
        
        self.data = self.model.createData()
        self.t = 0.0                    # time
        self.nv = nv = self.model.nv    # size of joint velocities vector
        self.na = na = nv-6         # number of actuated joints
        # Matrix S mapping control inputs to joint accelerations
        self.S = np.zeros((na, nv))
        self.S[0,3] = 1 # wire prismatic
        self.S[1,7] = 1 # hip pitch
        self.S[2,8] = 1 # hip roll
        self.S[3,9] = 1 # knee prismatic
        
        self.DISPLAY_T = 0.01     # refresh period for viewer
        self.display_counter = self.DISPLAY_T
        self.init(q, None)
                

    # Re-initialize the simulator
    def init(self, q0=None, v0=None):
        if q0 is not None:
            self.q = q0.copy()
            
        if(v0 is None):
            self.v = np.zeros(self.model.nv)
        else:
            self.v = v0.copy()
            
        self.dv = np.zeros(self.model.nv)


    def step(self, u, dt=None):
        if dt is None:
            dt = self.dt

        # compute all quantities needed for simulation
        pin.computeAllTerms(self.model, self.data, self.q, self.v)
        pin.updateFramePlacements(self.model, self.data)
        M = self.data.M         # mass matrix
        h = self.data.nle       # nonlinear effects (gravity, Coriolis, centrifugal)
        
        self.dv = np.linalg.solve(M, self.S.T.dot(u) - h)
        v_mean = self.v + 0.5*dt*self.dv
        self.v += self.dv*dt
        self.q = pin.integrate(self.model, self.q, v_mean*dt)

        self.t += dt
        return self.q, self.v


    def simulate(self, u, dt=0.001, ndt=1):
        ''' Perform ndt steps, each lasting dt/ndt seconds '''
        sub_dt = dt/ndt
        for i in range(ndt):
            self.q, self.v = self.step(u, sub_dt)

        if(self.use_viewer):
            self.display_counter -= dt
            if self.display_counter <= 0.0:
                self.display(self.q)

        return self.q, self.v

        
    def display(self, q):
        # update the visualization of the wire
        x_base = self.data.oMi[self.WIRE_BASE_JOINT_ID].translation
        wire_length = norm(x_base)
        self.gui.resizeArrow(self.WIRE_NODE_NAME, self.WIRE_RADIUS, wire_length)
        M_prismatic = self.data.oMi[self.WIRE_PRISMATIC_JOINT_ID]
        M_wire = M_prismatic * pin.SE3(pin.utils.rotate('y', -np.pi/2), np.zeros(3))
        self.gui.applyConfiguration(self.WIRE_NODE_NAME, pin.SE3ToXYZQUAT(M_wire).tolist())        

        self.viewer.display(q)
        
        self.display_counter = self.DISPLAY_T
        

        
if __name__=='__main__':    
    LINE_WIDTH = 50
    print("".center(LINE_WIDTH,'#'))
    print(" Test Climber ".center(LINE_WIDTH, '#'))
    print("".center(LINE_WIDTH,'#'), '\n')
    
    T_SIMULATION = 20       # total simulation time
    dt = 0.002              # time step
    ndt = 1               # number of simulation step per time step
    kp = np.array([30, 4])             # proportional gain for stabilizing wire prismatic joint
    kd = np.sqrt(kp)
    
    simu = ClimberSimulator(urdf)
    
    N = int(T_SIMULATION/dt)           # number of time steps
    tau    = np.zeros(simu.na)             # joint torques
    
    t = 0.0
#    time.sleep(3)
    for i in range(0, N):
        tau[0] = -kp[0] * simu.q[3] - kd[0]*simu.v[3]
        tau[3] = -kp[1] * simu.q[9] - kd[1]*simu.q[9]
        
        if(i%100==0):
            print("t", t)
            print("q", simu.q)
            print("tau", tau)
        
        time_start = time.time()
                
        # send joint torques to simulator
        simu.simulate(tau, dt, ndt)
#        time.sleep(0.1)
        t = i*dt
            
        time_spent = time.time() - time_start
        if(time_spent < dt): 
            time.sleep(dt-time_spent)
            
    print("Simulation finished")
    

