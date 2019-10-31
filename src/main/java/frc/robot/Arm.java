package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

class Arm {
	private final ControlMode kPercentOutput = ControlMode.PercentOutput;

	private final double PI = 3.14159,

	// Dimension macros
	LIFT_SHAFT_RADIUS = (0.875/39.37), LIFT_GEAR_RATIO = (100.0/3.0),
	ARM_GEAR_RATIO = 300.0, ARM_LENGTH = (15/39.37),

	ARM_MIN = 0,
	ARM_MAX = 1000,
	ARM_KP = 1.0,
	ARM_KI = 0.1,
	ARM_KD = 10,
	ARM_TOLERANCE = 1000,
	ARM_MAX_UP = 0.4,
	ARM_FF = 0.1,

	LIFT_MIN = 0,
	LIFT_MAX = 800000,
	LIFT_KP = 2.0,
	LIFT_KI = 0,
	LIFT_KD = 0,
	LIFT_TOLERANCE = 2000,
	LIFT_PID_RANGE = 200000,
	LIFT_INTEGRAL_MAX = 0.0,
	LIFT_MAX_DOWN = -0.2;

	public boolean armZeroed, liftZeroed;

	private PID armPid, liftPid;
	DoubleSolenoid clawSol,shootSol;
	private Timer timer;
	private boolean timing;
	TalonSRX arm0, arm1, lift0, lift1;
	Encoder armEnc, liftEnc;
	private double arm_integral, arm_prevError, lift_integral, lift_prevError, fine_integral;
	private boolean armFirstTime, liftFirstTime;
	private boolean openClaw;
	private boolean clawButtonPressed;
	DigitalInput liftTopLimit, liftBottomLimit, armTopLimit;

	Arm() {
		clawSol = new DoubleSolenoid(PORTS.SOL_CLAW_PCM, PORTS.SOL_CLAW_IN,PORTS.SOL_CLAW_OUT);
		shootSol = new DoubleSolenoid(PORTS.SOL_SHOOT_PCM, PORTS.SOL_SHOOT_IN,PORTS.SOL_SHOOT_OUT);
		// For gearbox motors
		arm0 = new TalonSRX(PORTS.ARM_TALON0);
		arm1 = new TalonSRX(PORTS.ARM_TALON1);
		lift0 = new TalonSRX(PORTS.LIFT_TALON0);
		lift1 = new TalonSRX(PORTS.LIFT_TALON1);
		armEnc = new Encoder(PORTS.ARM_ENCODER_A, PORTS.ARM_ENCODER_B, false, Encoder.EncodingType.k4X);
		armEnc.reset();
		liftEnc = new Encoder(PORTS.LIFT_ENCODER_A, PORTS.LIFT_ENCODER_B, false, Encoder.EncodingType.k4X);
		liftEnc.reset();

		openClaw = false;
		clawButtonPressed = false;

		liftTopLimit = new DigitalInput(PORTS.LIFT_TOP_LIMIT);
		liftBottomLimit = new DigitalInput(PORTS.LIFT_BOTTOM_LIMIT);
		armTopLimit = new DigitalInput(PORTS.ARM_TOP_LIMIT);

		armZeroed = false;
		liftZeroed = false;
	}

	/**
	 * Sets the arm motors to given input value
	 * as long as the limit switch is not triggered
	 */
	void armSet(double num) {
		if(num > 0 && !armTopLimit.get()) // If you're hitting the limit switch don't move
			num = 0;
		arm0.set(kPercentOutput, -num);
		arm1.set(kPercentOutput, -num);
	}

	/**
	 * Sets the lift motors to given input value
	 * as long as neither limit switch is not triggered
	 */
	void LiftSet(double num) {
		if((num < 0 && !liftBottomLimit.get()) || (num > 0 && !liftTopLimit.get())) // If you're hitting the  limit switch don't move
			num = 0;
		lift0.set(kPercentOutput, num); // Lift motors are wired in reverse
		lift1.set(kPercentOutput, num);
		SmartDashboard.putNumber("lift talon", num);
	}

	void reset(){
		armEnc.reset();
		liftEnc.reset();
		armZeroed = false;
		liftZeroed = false;
	}

	/**
	 * Sets the arm and lift using controller input
	 */
	void controllerMove(Joystick rightJoy, Joystick buttonBoard) {
		if (buttonBoard.getRawButton(BUTTONS.ZERO_IT) || buttonBoard.getPOV(0)/45 == 7){
			// Starting config
			armZeroed = false;
			liftZeroed = false;
		}
		SmartDashboard.putBoolean("Ready to Grab?", SmartDashboard.getNumber("Distance to Line", 1) <= 0.713);
		//armZeroed = true; liftZeroed = true;
		// Occasionally zeroing
		if (!liftBottomLimit.get()) liftEnc.reset();
		if (!armTopLimit.get()) armEnc.reset();

		SmartDashboard.putNumber("lift enc raw", liftEnc.getRaw());
		SmartDashboard.putNumber("arm enc raw", armEnc.getRaw());

		SmartDashboard.putBoolean("lift top limit", liftTopLimit.get());
		SmartDashboard.putBoolean("lift bot limit", liftBottomLimit.get());
		SmartDashboard.putBoolean("arm bot limit", armTopLimit.get());
		
		// Open and close claw
		if (rightJoy.getRawButton(BUTTONS.TOGGLE_CLAW)) {
			if(!clawButtonPressed) {
				openClaw = !openClaw;
				clawButtonPressed = true;
			}
			clawSol.set(openClaw ? DoubleSolenoid.Value.kReverse : DoubleSolenoid.Value.kForward);
		}
		else {
			clawSol.set(DoubleSolenoid.Value.kOff);
			clawButtonPressed = false;
		}
		if (buttonBoard.getRawButton(BUTTONS.DRIVER_OVERRIDE)) {
			if (buttonBoard.getRawButton(BUTTONS.CLAW_OPEN)) clawSol.set(DoubleSolenoid.Value.kReverse);
			else if (buttonBoard.getRawButton(BUTTONS.CLAW_CLOSE)) clawSol.set(DoubleSolenoid.Value.kForward);
		}

		// Moving the arm and lift
		double armVal = armEnc.getRaw();
		double liftVal = liftEnc.getRaw();

		// Manually move arm
		double arm_speed = 0.5;

		// First things first, let's zero the arm
		if (armTopLimit.get() && !armZeroed) { // If limit is not pressed and we have not zeroed yet
			armSet(0.25);//ARM_MAX_UP);
			armEnc.reset();
		} else {
			if(!armZeroed)
				armZeroed = true;
			if (buttonBoard.getRawButton(BUTTONS.ARM_DEFAULT))
				arm_speed = -buttonBoard.getRawAxis(BUTTONS.ARM_SPEED) / 2.0 + 0.5;
			if (buttonBoard.getRawButton(BUTTONS.ARM_UP))
				armSet(arm_speed);
			else if (buttonBoard.getRawButton(BUTTONS.ARM_DOWN))
				armSet(-arm_speed*0.5);
			else
				armSet(0.1);
		}

		// Manually move lift
		double lift_speed = 0.8;
		// if (buttonBoard.getRawButton(BUTTONS.LIFT_DEFAULT)) lift_speed = -buttonBoard.getRawAxis(BUTTONS.LIFT_SPEED) / 2.0 + 0.5;
		int level = buttonBoard.getPOV(0) / 45;
		if (level > 4) level = 0;

		// First things first, let's zero the lift
		if (liftBottomLimit.get() && !liftZeroed) { // If limit is not pressed and we have not zeroed yet
			LiftSet(LIFT_MAX_DOWN);
		} else {
			if(!liftZeroed){
				liftZeroed = true;
				liftEnc.reset();
			}
			if (buttonBoard.getRawButton(BUTTONS.LIFT_UP))
				LiftSet(lift_speed);
			else if (buttonBoard.getRawButton(BUTTONS.LIFT_DOWN))
				LiftSet(-lift_speed*0.25);
			else {
				// PID movement
				boolean fine = buttonBoard.getRawButton(BUTTONS.FINE_TUNE);
				switch (level) {
					case 1:
						BothPID(ARM_HEIGHTS.HATCH_LOAD, LIFT_HEIGHTS.HATCH_LOAD, fine); break;
					case 2:
						BothPID(ARM_HEIGHTS.HATCH_1, LIFT_HEIGHTS.HATCH_1, false); break;
					case 3:
						BothPID(ARM_HEIGHTS.HATCH_2, LIFT_HEIGHTS.HATCH_2, false); break;
					case 4:
						BothPID(ARM_HEIGHTS.HATCH_3, LIFT_HEIGHTS.HATCH_3, false); break;
					default:
						LiftSet(0.075);
				}
			}
		}
	}

	/**
	 * Runs PID for the arm and Lift
	 */
	boolean BothPID(double goalA, double goalL, boolean fine) {
		boolean pidA = armPID(goalA, fine);
		boolean pidL = LiftPID(goalL, fine);
		return pidA && pidL;
	}

	/**
	 * Moves the arm to the desired goal height
	 */
	boolean armPID(double goal, boolean fine) {
		SmartDashboard.putBoolean("Fine Tune?", false);
		if (armFirstTime) {
			arm_integral = 0;
			fine_integral = 0;
			arm_prevError = 0;
			armFirstTime = false;
		}
		if (fine) { // Fine tune based on camera movement, needs different PID values
			return armPIDFine();
		}
		double error = goal - armEnc.getRaw();
		SmartDashboard.putNumber("arm PID error", error);
		if (Math.abs(error) < ARM_TOLERANCE) {
			// End the function right here
			armFirstTime = true;
			arm_integral = 0;
			fine_integral = 0;
			armSet(0.1);
			return true;
		}
		error /= 200000.0;
		arm_integral += error;
		if (arm_integral > 0.3/ARM_KI) arm_integral = 0.3/ARM_KI;
		if (arm_integral < -0.3/ARM_KI) arm_integral = -0.3/ARM_KI;
		double derivative = error - arm_prevError;
		SmartDashboard.putNumber("arm PID d", derivative);
		SmartDashboard.putNumber("arm PID i", arm_integral);
		double speed = error * ARM_KP + arm_integral * ARM_KI + derivative * ARM_KD + ARM_FF;
		arm_prevError = error;
		if (speed > 1) speed = 1;
		if (speed < -0.25) speed = -0.25;
		SmartDashboard.putNumber("arm PID speed", speed);
		armSet(speed);
		return false;
	}

	boolean armPIDFine(){
		double kP = 3.0;//7.0
		double kI = 0.0;//0.05
		double kD = 0;
		double kFF = -0.1;

		double offset = SmartDashboard.getNumber("vertical_offset",0.0);
		double goal = 0.0335; // Target vision offset
		double error = goal - offset; // in meters
		SmartDashboard.putBoolean("Fine Tune?", true);
		SmartDashboard.putNumber("fine tune arm PID error", error);
		if (Math.abs(error) < 0.000001) {
			// End the function right here
			armFirstTime = true;
			fine_integral = 0;
			armSet(0.1);
			return true;
		}
		fine_integral += error;
		// if (fine_integral > 0.3/kI) fine_integral = 0.3/kI;
		// if (fine_integral < -0.3/kI) fine_integral = -0.3/kI;
		double derivative = error - arm_prevError;
		SmartDashboard.putNumber("fine tune arm PID d", derivative);
		SmartDashboard.putNumber("fine tune arm PID i", fine_integral);
		double speed = error * kP + fine_integral * kI + derivative * kD + kFF;
		speed *= -1;
		arm_prevError = error;
		if (speed > 1) speed = 1;
		if (speed < -1) speed = -1;
		SmartDashboard.putNumber("fine tune arm PID speed", speed);
		armSet(speed);
		return false;
	}

	/**
	 * Moves the lift to the desired goal height
	 */

	boolean LiftPID(double goal, boolean fine) {
		if (liftFirstTime) {
			lift_integral = 0;
			lift_prevError = 0;
			liftFirstTime = false;
		}
		double error = goal - liftEnc.getRaw();
		if (Math.abs(error) < LIFT_TOLERANCE) {
			// End the function right here
			liftFirstTime = true;
			lift_integral = 0;
			LiftSet(0.0);
			return true;
		}
		if (Math.abs(error) > LIFT_PID_RANGE) {
			LiftSet(error > 0 ? 0.8 : -0.5);
			return false;
		}

		SmartDashboard.putNumber("error", error);

		error /= LIFT_PID_RANGE;

		lift_integral += error;
		if (lift_integral > LIFT_INTEGRAL_MAX) lift_integral = LIFT_INTEGRAL_MAX;
		else if (lift_integral < -LIFT_INTEGRAL_MAX) lift_integral = -LIFT_INTEGRAL_MAX;

		double derivative = error - lift_prevError;

		double speed = error * LIFT_KP + lift_integral * LIFT_KI + derivative * LIFT_KD + .05;
		lift_prevError = error;
		if (speed > 0.8) speed = 0.8;
		if (speed < -0.5) speed = -0.5;
		// if (speed >= 0 && speed < 0.25) speed = 0.25;
		// if (speed < 0 && speed > -0.25) speed = -0.25;
		SmartDashboard.putNumber("PID speed", speed);
		LiftSet(speed);
		return false;
	}

	void Grab(){
		clawSol.set(DoubleSolenoid.Value.kForward);
		openClaw = false;
	}
}

class LIFT_HEIGHTS {
	public final static int HATCH_LOAD = 272644,
	HATCH_1 = 487238, // 672001
	HATCH_2 = 487238,
	HATCH_3 = 996858;
}

class ARM_HEIGHTS {
	public final static int HATCH_LOAD = -201284, // 368179.0
	HATCH_1 = -252979 ,//-274796 too low
	HATCH_2 = -97123,
	HATCH_3 = -76553;
}