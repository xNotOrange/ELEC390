# pid_controller.py
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def update(self, error, dt):
        P_out = self.Kp * error
        self.integral += error * dt
        I_out = self.Ki * self.integral
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        D_out = self.Kd * derivative
        output = P_out + I_out + D_out
        self.prev_error = error
        return output
