package frc.robot;

class PID {
	private double Kp, Ki, Kd, tolerance;
	private double prev_error, integral;

	PID(double _Kp, double _Ki, double _Kd, double tol) {
		Kp = _Kp;
		Ki = _Ki;
		Kd = _Kd;
		tolerance = tol;
		prev_error = 0;
		integral = 0;
	}

	public void reset() {
		prev_error = 0;
		integral = 0;
	}

	public double getPID(double current, double goal) {
		double error = goal - current;
		if (Math.abs(error) < tolerance) {
			return -1;
		}
		double derivative = error - prev_error;
		integral += error;
		double speed = Kp * error + Ki * integral + Kd * derivative;
		prev_error = error;
		return speed;
	}
}