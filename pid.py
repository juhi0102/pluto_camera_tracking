class PID:
    def _init_(self):
        self.Kp = 0
        self.Ki = 0
        self.Kd = 0
        self.setpoint = 0
        self.dt = 0
        
        self.prev_error = 0
        self.integral = 0  # Initialize integral here

    def set_parameters(self, Kp, Ki, Kd, setpoint, dt):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.dt = dt
        self.integral = 0
        self.prev_error = 0

    def update(self, feedback):
        error = self.setpoint - feedback
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.prev_error = error

        return output
