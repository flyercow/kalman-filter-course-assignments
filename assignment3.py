import numpy as np
from sim.sim2d import sim_run

# Simulator options.
options = {}
options['FIG_SIZE'] = [8,8]

options['DRIVE_IN_CIRCLE'] = True
# If False, measurements will be x,y.
# If True, measurements will be x,y, and current angle of the car.
# Required if you want to pass the driving in circle.
options['MEASURE_ANGLE'] = False
options['RECIEVE_INPUTS'] = False

class KalmanFilter:
    def __init__(self):
        # Initial State
        self.x = np.matrix([[0.],
                            [0.],
                            [0.],
                            [0.]])

        # Uncertainity Matrix
        self.P = np.matrix([[1000.,    0.,    0.,    0.],
                            [   0., 1000.,    0.,    0.],
                            [   0.,    0., 1000.,    0.],
                            [   0.,    0.,    0., 1000.]])

        # Next State Function
        self.F = np.matrix([[1., 0., 1., 0.,],
                            [0., 1., 0., 1.,],
                            [0., 0., 1., 0.,],
                            [0., 0., 0., 1.,]])

        # Measurement Function
        self.H = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.]])
        # Measurement Uncertainty
        self.R = np.matrix([[100,   0],
                            [  0, 100]])

        # Identity Matrix
        self.I = np.matrix([[1., 0., 0., 0.],
                            [0., 1., 0., 0.],
                            [0., 0., 1., 0.],
                            [0., 0., 0., 1.]])
    def predict(self, dt):
        # dt          = t - self.prev_time
        self.F[0,2] = dt
        self.F[1,3] = dt
        self.x      = self.F * self.x
        self.P      = self.F * self.P * np.transpose(self.F)
        return [self.x[0], self.x[1]]
    
    def measure_and_update(self,measurements, dt):
        # dt = t - self.prev_time
        self.F[0,2] = dt
        self.F[1,3] = dt
        Z = np.matrix(measurements)
        y = np.transpose(Z) - self.H * self.x
        S = self.H * self.P * np.transpose(self.H) + self.R
        K = self.P * np.transpose(self.H) * np.linalg.inv(S)
        self.x = self.x + K*y
        self.P = (self.I - K*self.H)*self.P
        self.P[0,0] += 0.1
        self.P[1,1] += 0.1
        self.P[2,2] += 0.1
        self.P[3,3] += 0.1
        # self.v = self.x[1,0]
        # self.prev_time = t
        return [self.x[0], self.x[1]]

    def recieve_inputs(self, u_steer, u_pedal):
        return

sim_run(options,KalmanFilter)
