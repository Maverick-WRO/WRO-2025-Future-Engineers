class AckermannController:
    def __init__(self, wheelbase):
        self.wheelbase = wheelbase

    def steering_angle(self, curvature):
        return curvature * self.wheelbase
