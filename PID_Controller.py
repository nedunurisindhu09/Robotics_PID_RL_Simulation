import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    def __init__(self, kp, ki, kd, dt=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, actual_value):
        error = setpoint - actual_value
        self.integral += error * self.dt
        derivative = (error - self.prev_error) / self.dt

        output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        self.prev_error = error
        return output


# Example test
if __name__ == "__main__":
    pid = PIDController(kp=2.0, ki=0.5, kd=1.0)
    setpoint = 10.0
    actual = 0.0

    results = []
    for _ in range(200):
        control = pid.compute(setpoint, actual)
        actual += control * 0.05  # simulate system response
        results.append(actual)

    plt.plot(results, label='System Response')
    plt.axhline(setpoint, color='r', linestyle='--', label='Setpoint')
    plt.xlabel('Time steps')
    plt.ylabel('Output')
    plt.title('PID Controller Response')
    plt.legend()
    plt.show()
