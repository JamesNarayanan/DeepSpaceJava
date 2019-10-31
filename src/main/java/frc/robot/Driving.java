package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;

import com.kauailabs.navx.frc.AHRS;

class Driving {
	private final double VEL_HIGH_THRESHOLD = 5600, VEL_LOW_THRESHOLD = 1.00, VEL_JOY_THRESHOLD = 1.00,

			WHEEL_BASE = 0.6858, WHEEL_RADIUS = 0.1, GEAR_RATIO_HIGH = 12.6176, GEAR_RATIO_LOW = 32.5000,

			PI = 3.14159265;

	private Arm arm;
	private CANSparkMax sparkRF, sparkRB, sparkLF, sparkLB;
	private CANEncoder encRF, encRB, encLF, encLB;
	private Compressor comp;
	private DoubleSolenoid soll, solr;
	private AHRS navX;
	private int gear;
	private double leftStick = 0, rightStick = 0;
	private double prev;
	private Timer timer;
	private double x = 0.0, y = 0.0;
	private double integratedGyro = 0, integratedEncoder = 0;

	private boolean gearButtonPressed, shiftUp;
	private CANPIDController pidRF, pidRB, pidLF, pidLB;
	private int assistStep;

	private double prevOffset, offsetIntegral;
	private double voltageL, voltageR;
	private double prevDistance;
	private double driveMultiplier;

	private boolean assistedPressed;
	private double v, omega;

	public Driving() {
		sparkLF = new CANSparkMax(PORTS.SPARK_LF, MotorType.kBrushless);
		sparkLB = new CANSparkMax(PORTS.SPARK_LB, MotorType.kBrushless);
		sparkRF = new CANSparkMax(PORTS.SPARK_RF, MotorType.kBrushless);
		sparkRB = new CANSparkMax(PORTS.SPARK_RB, MotorType.kBrushless);
		sparkRF.setInverted(true);
		sparkRB.setInverted(true);

		// Init encoders
		encLF = new CANEncoder(sparkLF);
		encLB = new CANEncoder(sparkLB);
		encRF = new CANEncoder(sparkRF);
		encRB = new CANEncoder(sparkRB);

		pidLF = new CANPIDController(sparkLF);
		pidLB = new CANPIDController(sparkLB);
		pidRF = new CANPIDController(sparkRF);
		pidRB = new CANPIDController(sparkRB);

		setPIDConstants(pidLF);
		setPIDConstants(pidLB);
		setPIDConstants(pidRF);
		setPIDConstants(pidRB);

		// Init gyro
		navX = new AHRS(SerialPort.Port.kUSB1);

		comp = new Compressor(PORTS.COMPRESSOR);
		comp.start();
		// comp.Stop();
		soll = new DoubleSolenoid(PORTS.SOL_SHIFT_L_PCM, PORTS.SOL_SHIFT_L_IN, PORTS.SOL_SHIFT_L_OUT);
		solr = new DoubleSolenoid(PORTS.SOL_SHIFT_R_PCM, PORTS.SOL_SHIFT_R_IN, PORTS.SOL_SHIFT_R_OUT);
		gear = GEAR.HIGH;

		gearButtonPressed = false;
		shiftUp = false; // Starts in low gear

		driveMultiplier = 0.4;
		assistedPressed = false;
	}

	public void reset() {
		// navX.reset();
		prev = 0;
		timer.start();
		integratedEncoder = 0;
	}

	/**
	 * Uses controller input to move the robot
	 * This is called during teleop periodic
	 */
	public void controllerMove(Joystick leftJoy, Joystick rightJoy, Joystick buttonBoard) {
		SmartDashboard.putNumber("left velocity", encLF.getVelocity());
		// filled pressure?
		SmartDashboard.putBoolean("pressurized?",	comp.getPressureSwitchValue());
		
		if (buttonBoard.getRawButton(BUTTONS.DRIVING_ASSIST)) {
			assistedDriving();
			SmartDashboard.putBoolean("send_nav_goal", true);
			assistedPressed = true;
			return;
		} else {
			SmartDashboard.putBoolean("send_nav_goal", false);
			assistedPressed = false;
		}
		if (buttonBoard.getRawButton(BUTTONS.ASSIST_LITE)) {
			assistedDrivingLite();
			SmartDashboard.putBoolean("Assisted?", true);
			return;
		} 
		else SmartDashboard.putBoolean("Assisted?", false);
		if (buttonBoard.getPOV(0)/45 == 5){ // HAB 1 to 2 button
			return;
		}
		// Tank drive
		if(leftJoy.getRawButton(BUTTONS.TOGGLE_SHIFT) && driveMultiplier < 1){
			driveMultiplier += 0.01;
		} else if (driveMultiplier > 0.4) driveMultiplier -= 0.01;
		
		leftStick = -leftJoy.getRawAxis(1)*driveMultiplier; 
		rightStick = -rightJoy.getRawAxis(1)*driveMultiplier;
		if (Math.abs(leftStick) < 0.04) leftStick = 0.0;
		if (Math.abs(rightStick) < 0.04) rightStick = 0.0;
		drive(leftStick, rightStick, buttonBoard.getRawButton(BUTTONS.DRIVER_OVERRIDE));
		// Transmission
		if (buttonBoard.getRawButton(BUTTONS.TOGGLE_CLAW)) { // TOGGLE_CLAW button
			if(!gearButtonPressed) {
				shiftUp = !shiftUp;
				gearButtonPressed = true;
			}
			shift(shiftUp ? GEAR.HIGH : GEAR.LOW);
		} else { // Off
			//shift(GEAR.STAY);
			gearButtonPressed = false;
		}
		if (buttonBoard.getRawButton(BUTTONS.DRIVER_OVERRIDE)) {
			if (buttonBoard.getRawButton(BUTTONS.SHIFT_UP)) shift(GEAR.HIGH); 
			else if (buttonBoard.getRawButton(BUTTONS.SHIFT_DOWN)) shift(GEAR.LOW);
		}
	}

	/**
	 * Sets the motor controllers to move the robot
	 */
	public void drive(double left, double right, boolean sensitive) {

		// sparkLF.set(left);
		// sparkLB.set(left);
		// sparkRF.set(-right);
		// sparkRB.set(-right);
		if (sensitive){
			left = left*left*left;
			right = right*right*right;
		}
		sparkLF.set(left);
		sparkLB.set(left);
		sparkRF.set(right);
		sparkRB.set(right);
		voltageL = left;
		voltageR = right;
		sendOdometry();
	}

	/**
	 * Shifts gears uMath.sing the given parameter g
	 */
	public void shift(int g) {
		if (g == GEAR.LOW) { // Shift down
			soll.set(DoubleSolenoid.Value.kReverse);
			solr.set(DoubleSolenoid.Value.kReverse);
			gear = GEAR.LOW;
		} else if (g == GEAR.HIGH) { // Shift up
			soll.set(DoubleSolenoid.Value.kForward);
			solr.set(DoubleSolenoid.Value.kForward);
			gear = GEAR.HIGH;
		} else { // Stay
			soll.set(DoubleSolenoid.Value.kOff);
			solr.set(DoubleSolenoid.Value.kOff);
		}
	}

	public void AutoShift() {
		// First, let's figure out the typical ratio of velocity to joystick value
		double speeds[] = {encLB.getVelocity(), encLF.getVelocity(), encRB.getVelocity(), encRF.getVelocity()};
		double sum = 0.0;
		// double len = sizeof(speeds)/sizeof(speeds[0]); // Taking the size of a double should always be 8
		double len = speeds.length/8;

		for (int i = 0; i < len; i++) sum += speeds[i];
		double speed = sum / len;
		SmartDashboard.putNumberArray("Speeds", speeds);
		SmartDashboard.putNumber("Average Speed", speed);
		double joy = (Math.abs(leftStick) + Math.abs(rightStick)) / 2.0;
		SmartDashboard.putNumber("Average Joy", joy);
		double ratio = (joy == 0) ? 0 : speed / joy;
		SmartDashboard.putNumber("Speed-Joy", ratio);

		// Shift up if we're going fast
		if (speed > VEL_HIGH_THRESHOLD && gear == GEAR.LOW) {
			shift(GEAR.HIGH);
		}

		// SmartDashboard.putNumber("angle",navX.getAngle());
	}

	public void setPIDConstants(CANPIDController p) {
		// Default PID coefficients
		p.setP(4e-8);//5e-5//4e-8
		p.setI(2.5e-7);//1e-6
		p.setD(4e-9);
		p.setIZone(0);
		p.setFF(0.0);//0.000156
		p.setOutputRange(-1, 1);
	}

	public void velocityPID(double linear, double angular) { // speed in m/s and rad/s
		double ratio = (gear == GEAR.HIGH) ? GEAR_RATIO_HIGH : GEAR_RATIO_LOW;
		double left_target = linear - angular * WHEEL_BASE / 2.0;
		double right_target = linear + angular * WHEEL_BASE / 2.0;
		left_target *= 1.0 / (2 * PI * WHEEL_RADIUS) * ratio * 60.0;
		right_target *= 1.0 / (2 * PI * WHEEL_RADIUS) * ratio * 60.0; // Convert robot m/s to motor rpm
		SmartDashboard.putNumber("left_target",left_target);
		SmartDashboard.putNumber("right_target",right_target);

		double voltage = sparkLF.getBusVoltage();
		pidLF.setReference(left_target, ControlType.kVelocity);
		pidLB.setReference(left_target, ControlType.kVelocity);
		pidRF.setReference(right_target, ControlType.kVelocity);
		pidRB.setReference(right_target, ControlType.kVelocity);//, 0, voltage * voltageR);

		SmartDashboard.putNumber("left_vel", encLF.getVelocity());
		SmartDashboard.putNumber("right_error",right_target-encRF.getVelocity());

		sendOdometry();

	}

	public void sendOdometry() {
		// Calculate linear and angular velocity for ROS feedback
		double ratio = (gear == GEAR.HIGH) ? GEAR_RATIO_HIGH : GEAR_RATIO_LOW;
		// integratedEncoder += encLF.getVelocity()* (2 * PI * WHEEL_RADIUS) / 60.0 / ratio *.02;
		// Prateek, shouldn't we average the left and right velocity for this in order to get the linear velocity?

		double velLeft = encLF.getVelocity() * (2 * PI * WHEEL_RADIUS) / 60.0 / ratio;
		double velRight = encRF.getVelocity() * (2 * PI * WHEEL_RADIUS) / 60.0 / ratio;
		double linVel = (velLeft + velRight) / 2.0;
		double angVel = (velRight - velLeft) / WHEEL_BASE;

		//integratedEncoder += angVel*timer.get();

		double gyro = navX.getYaw() * PI/180;

		x += linVel*timer.get()*Math.cos(-gyro);
		y += linVel*timer.get()*Math.sin(-gyro);
		
		SmartDashboard.putNumber("linear_velocity", linVel);
		SmartDashboard.putNumber("angle", -gyro);
		SmartDashboard.putNumber("dt", timer.get());
		SmartDashboard.putNumber("angular_velocity", -navX.getRate());
		SmartDashboard.putNumber("x", x);
		SmartDashboard.putNumber("y", y);

		//SmartDashboard.putNumber("positionLF", encLF.getPosition()/ratio*2*PI*WHEEL_RADIUS);
		//SmartDashboard.putNumber("position RF", encRF.getPosition()/ratio*2*PI*WHEEL_RADIUS);

		// SmartDashboard.putNumber("angular_velocity_encoder", angVel);
		// SmartDashboard.putNumber("linear_velocity_gyro", navX.getVelocityX());
		// SmartDashboard.putNumber("left encoder", encLF.getVelocity());
		// SmartDashboard.putNumber("right encoder", encRF .getVelocity());

		// integratedGyro += navX.getRate()*timer.get();
		// SmartDashboard.putNumber("integrated angular_velocity", integratedEncoder*180/PI);
		// SmartDashboard.putNumber("integrated angular_velocity_gyro", integratedGyro*180/PI);
		// SmartDashboard.putNumber("integrated linvel", integratedEncoder);
		integratedEncoder += linVel *timer.get();
		// SmartDashboard.putNumber("x integrated", integratedEncoder);


		timer.reset();
	}

		/*
		* public void assistedDriving(){ if
		* (!SmartDashboard.getBoolean("send_nav_goal", false)) { assistStep = 0;
		* prevOffset = 0; offsetIntegral = 0; } velocityPID(
		* SmartDashboard.getNumber("target_linear_velocity", 0),
		* SmartDashboard.getNumber("target_angular_velocity", 0) ); }
		*/

		// with initial align
	public void assistedDrivingLite() {
		double offset = SmartDashboard.getNumber("horizontal_offset_normalized", 0.0);
		double angle = SmartDashboard.getNumber("Angle of Line", 0.0); // In degrees
		double distance = SmartDashboard.getNumber("Distance to Line", 0.0) / Math.cos(angle * PI / 180.0);
		distance -= 0.7366;
		if (!SmartDashboard.getBoolean("Assisted?", false)) {
			assistStep = 0;
			prevOffset = 0;
			offsetIntegral = 0;
			prevDistance = distance; 
		}
		voltageL = 0;
		voltageR = 0;
		SmartDashboard.putNumber("Assist Step", assistStep);
		if (assistStep >= 2) {
			drive(0.0,0.0, false);
			if (assistStep <= 27){
				// arm.Grab();
				assistStep += 1;
			}
			return;
		}
		voltageL *= .9;
		voltageR *= .9;
		double currSpeed = SmartDashboard.getNumber("linear_velocity", 0);
		currSpeed = currSpeed == 0 ? 0.1 : currSpeed;
		if (distance - prevDistance > 0.6){
			distance = prevDistance + (distance - prevDistance)*0.2;
		}
		prevDistance = distance;
		double minDistance = currSpeed * 0.65; // 0.12;
		double angSpeed = 0;
		double linSpeed = 0;
		if (assistStep == 0 || distance >= .07) {
			if (Math.abs(offset) > 0.05) {
				double kP = 0.4;
				double kI = 0.06;
				double kD = 0.6;
				if (assistStep == 1){
					kP = 2; // 2
					kI = 0;
					kD = 1;
				}
				offsetIntegral += offset;
				angSpeed = kP * offset + kI * offsetIntegral + kD * (offset - prevOffset);
				prevOffset = offset;
			}
		}
		SmartDashboard.putNumber("rotational velcoity lite", angSpeed);
		if (assistStep == 1) {
			if (distance > minDistance || true) {
				double kP_lin = 0.65;
				double max_speed = 1;
				linSpeed = kP_lin * distance * distance;
				if (linSpeed > max_speed) linSpeed = max_speed;
			} else {
				assistStep++;
				// Stop! Quickly!
				double kP_stop = 0.1;
				double lSpeed = encLF.getVelocity();
				double rSpeed = encRF.getVelocity();
				drive(-kP_stop * lSpeed, -kP_stop * rSpeed, false);
				return;
			}
		}
		if(assistStep == 0 && Math.abs(offset) < 1) {
			assistStep++;
			//drive(0,0);
			prevOffset = 0;
			offsetIntegral = 0;
			return;
		}
		velocityPID(linSpeed, -angSpeed);
	}

		// with initial align
		/*
		* public void assistedDriving() { if
		* (!SmartDashboard.getBoolean("Assisted?", false)) { assistStep = 0;
		* prevOffset = 0; offsetIntegral = 0; }
		* SmartDashboard.putNumber("Assist Step", assistStep);
		* 
		* double offset = SmartDashboard.getNumber("horizontal_offset_normalized",
		* 0.0); double angle = SmartDashboard.getNumber("Angle of Line", 0.0); // In
		* degrees double distance = SmartDashboard.getNumber("Distance to Line", 0.0)
		* / Math.cos(angle * PI / 180.0); distance -= 29.0 / 39.37; double minDistance =
		* 0.04; double angSpeed = 0; double linSpeed = 0; if (assistStep == 0 ||
		* distance >= 1) { if (Math.abs(offset) > 0.05) { double kP = 0.4; double kI =
		* 0.06; double kD = 0.6; if (assistStep == 1){ kP = 0.4; kI = 0; kD = 10.0; }
		* offsetIntegral += offset; angSpeed = kP * offset + kI * offsetIntegral + kD *
		* prevOffset; prevOffset = offset; } } if (assistStep == 1) { if (distance >
		* minDistance) { double kP_lin = 1.5; double max_speed = 1.0; linSpeed = kP_lin
		* * distance * distance; if (linSpeed > max_speed) linSpeed = max_speed; } else
		* { assistStep++; drive(0, 0); return;; } } if(assistStep == 0 && Math.abs(offset)
		* < .05) { assistStep++; drive(0,0); prevOffset = 0; offsetIntegral = 0;
		* return; } velocityPID(linSpeed, -angSpeed); }
		*/

	public void assistedDriving(){
		// Source: Hadzic math
		double x = SmartDashboard.getNumber("arm y", 0); // From camera, intentionally swapped for ROS
		double y = SmartDashboard.getNumber("arm x", 0);
		if (!assistedPressed){
			double theta = SmartDashboard.getNumber("Angle of Line", 0); // From lidar, angle for rotation
			double d = Math.sqrt(x*x + y*y); // Distance to hatch
			double phi = Math.atan(y/x); // Angle for translation
			double R = d / 2.0 / Math.cos(theta + phi); // Turn radius
			v = 0.5; // Linear speed is constant
			omega = v / R; // Angular speed is dependent on linear speed
			assistedPressed = true;
			velocityPID(v, omega); // Go time!
		}
		SmartDashboard.putNumber("target lin vel:", v); 
		SmartDashboard.putNumber("target ang vel:", omega);
	}

	private class GEAR {
		public final static int LOW = 1, HIGH = 2, STAY = 0;
	}
}