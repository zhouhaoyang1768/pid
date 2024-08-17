void tune(double error, double& Kp, double& Ki, double& Kd)
{
	static const int N = 2;
	static double table[N][5] = {
		// lower   upper     Kp      Ki      Kd
		// bound   bound
		{      0,    100,   0.1,   0.03,    0.1},
		{    100,    200,   0.2,   0.05,    0.1}
	};
	for (int i = 0; i < N; ++i)                             // O (N) where N a constant
	{
		if (error >= table[i][0] && error < table[i][1])
		{
			Kp = table[i][2];
			Ki = table[i][3];
			Kd = table[i][4];
			return;
		}
	}

}

double abs(double n)
{
	if (n >= 0)
		return n;
	else
		return -n;
}

class PID
{
	double Kp, Ki, Kd;
	double tolerance;
public:
	PID(double Kp, double Ki, double Kd, double tolerance) : Kp(Kp), Ki(Ki), Kd(Kd), tolerance(tolerance) {

	}




	/*
	 * get_pv gets the current process variable
	 * set sets the actuator
	 * set_point is the target value for process variable
	 *
	 * Time: O(1), additions, multiplications, if statements, getter/setter function calls
	 * Space: O(1), 84 bytes
	 */
	void begin(double (*get_pv)(), void (*set)(double), double set_point)
	{
		double I = 0;
		double last_error = 0;

		double Kp = this->Kp;
		double Ki = this->Ki;
		double Kd = this->Kd;

		int count = 0;


		tune(set_point - get_pv(), Kp, Ki, Kd);
		int n = 0;
		while (count < 10)
		{
			++n;
			double current = get_pv();
			double error = set_point - current;

			if (abs(error) < tolerance) ++count;
			else count = 0;

			double P = error;
			I = I + error;
			double D = error - last_error;



			if (current > set_point && I > 0) I = 0; // If pv overshoots and I is positive, reduce to 0
			if (current < set_point && I < 0) I = 0; // If pv overshoots below and I is negative, reduce to 0
			if (abs(current - set_point) < tolerance) I = 0; // If pv is close to set point, reduce I to 0

			last_error = error;
			double val = (Kp * P + Ki * I + Kd * D);
			set(val);
		}

	}
};

/*
 * Returns the process variable from sensors
 */
double get_pv() {
	// return simulator.pv;
}


/*
 * Sets the output
 */
void set_output(double val)
{
	//simulator.set_power(val);
}

int main()
{
	double Kp = 1;
	double Ki = 0.1;
	double Kd = 0.5;
	double tolerance = 1;
	PID pid(Kp, Ki, Kd, tolerance);


	double set_point = 45;
	pid.begin(get_pv, set_output, set_point);
}