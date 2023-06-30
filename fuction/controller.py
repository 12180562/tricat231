class controller:
    def __init__(self,Kp,Kd):
        # PD gain
        self.K_P = Kp
        self.K_D = Kd

        self.C_P = 0
        self.C_D = 0

    def PD_controller(self, error, dt):

        de = error - self.previous_error

        self.Cp = error
        self.Cd = de / dt

        self.previous_error = error

        return ((self.K_P * self.Cp) + (self.K_D * self.Cd))