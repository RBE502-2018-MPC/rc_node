

class PID:
    # PID Control based on example from:
    # https://github.com/JunshengFu/PID-controller
    #
    # This algorithm, Twiddle, could be useful for tuning the PID parameters:
    # https://martin-thoma.com/twiddle/

    def __init__(self, kp, ki, kd):
        self.p_error = 0
        self.i_error = 0
        self.d_error = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def update_error(self, error):
        # Error is the current cross track error
        prior_error = self.p_error
        self.p_error = error
        self.i_error = self.i_error + error
        self.d_error = error - prior_error

    def output_steering(self):
        # We should add min/max voltages and an indicator for steering control saturation
        return - self.kp*self.p_error - self.ki*self.i_error - self.kd*self.d_error
