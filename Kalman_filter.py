from numpy.linalg import inv
import numpy as np

class KalmanFilter:
    def __init__(self, initial_state, initial_covariance, process_noise, gps_noise):
        """
        Enhanced Kalman Filter for fusing odometry and GPS data
        
        Parameters:
        initial_state: [x, y, theta, v, omega]  (m, m, rad, m/s, rad/s)
        initial_covariance: Initial state covariance matrix (5x5)
        process_noise: Process noise matrix Q (5x5)
        gps_noise: GPS measurement noise matrix R (2x2)
        """
        self.state = initial_state
        self.covariance = initial_covariance
        self.Q = process_noise
        self.R = gps_noise
        self.d_ = 1.765  # Wheelbase (should match your odometry node)
        
        # State transition matrix (will be updated dynamically)
        self.F = np.eye(5)
        
        # Measurement matrix (GPS only measures x,y)
        self.H = np.array([
            [1, 0, 0, 0, 0],
            [0, 1, 0, 0, 0]
        ])
    
    def predict(self, dt, odom_data):
        """
        Prediction step using consistent odometry motion model
        
        Parameters:
        dt: Time step (s)
        odom_data: [x, y, theta, v, omega] from odometry
        """
        theta = self.state[2]
        v = odom_data[3]
        omega = odom_data[4]
        
        # Update state using the same model as your odometry node
        if abs(omega) < 1e-6:
            # Runge-Kutta approximation for small omega
            self.state[0] += v * dt * np.sin(theta + (omega * dt)/2)
            self.state[1] += v * dt * np.cos(theta + (omega * dt)/2)
            self.state[2] += omega * dt
        else:
            # Exact solution for larger omega
            self.state[0] += (v/omega) * (np.cos(theta) - np.cos(theta + omega * dt))
            self.state[1] += (v/omega) * (np.sin(theta + omega * dt) - np.sin(theta))
            self.state[2] += omega * dt
        
        # Keep velocity states
        self.state[3] = v
        self.state[4] = omega
        
        # defining state transition matrix
        if abs(omega) < 1e-6:
            self.F = np.array([
                [1, 0, v*dt*np.cos(theta), dt*np.sin(theta), 0],
                [0, 1, -v*dt*np.sin(theta), dt*np.cos(theta), 0],
                [0, 0, 1, 0, dt],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1]
            ])
        else:
            cth = np.cos(theta)
            sth = np.sin(theta)
            cthw = np.cos(theta + omega*dt)
            sthw = np.sin(theta + omega*dt)
            
            self.F = np.array([
                [1, 0, (v/omega)*(sth - sthw), (1/omega)*(cth - cthw), (v/omega**2)*(cthw - cth) + (v*dt/omega)*sthw],
                [0, 1, (v/omega)*(cthw - cth), (1/omega)*(sthw - sth), (v/omega**2)*(sthw - sth) - (v*dt/omega)*cthw],
                [0, 0, 1, 0, dt],
                [0, 0, 0, 1, 0],
                [0, 0, 0, 0, 1]
            ])
        
        # covariance prediction
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
    
    def update(self, gps_measurement):
        """
        Update step using GPS measurement
        
        Parameters:
        gps_measurement: [x, y] position (m)
        """
        # Measurement residual
        y = gps_measurement - self.H @ self.state
        
        # Kalman gain
        S = self.H @ self.covariance @ self.H.T + self.R
        K = self.covariance @ self.H.T @ inv(S)
        
        # State update
        self.state += K @ y
        
        # Covariance update
        I = np.eye(self.covariance.shape[0])
        self.covariance = (I - K @ self.H) @ self.covariance
    
    def get_state(self):
        """Return current state estimate"""
        return self.state.copy()
    
    def get_position(self):
        """Return current position estimate"""
        return self.state[:2].copy()