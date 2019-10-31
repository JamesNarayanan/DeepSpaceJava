package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

class Intake {
	private final ControlMode kPercentOutput = ControlMode.PercentOutput;

	private final double INTAKE_KP = 2,
	INTAKE_KI = 0.05,
	INTAKE_KD = 0.0,
	PID_RANGE = 200000,
	TOLERANCE = 2000,
	INTAKE_ROTATION_MAX = -0.5;


	public boolean intakeZeroed;

	private TalonSRX intake, intakeUp0, intakeUp1;
	private Encoder encoder;
	private DigitalInput topLimit;
	private double integral, prevError;
	private boolean firstTime;

	Intake() {
		intake = new TalonSRX(PORTS.TALON_INTAKE);
		intakeUp0 = new TalonSRX(PORTS.TALON_INTAKE_UP0);
		intakeUp1 = new TalonSRX(PORTS.TALON_INTAKE_UP1);
		encoder = new Encoder(PORTS.INTAKE_ENC_A,PORTS.INTAKE_ENC_B, true, Encoder.EncodingType.k4X);
		topLimit = new DigitalInput(PORTS.INTAKE_TOP_LIMIT);

		intakeZeroed = false;
	}

	void reset(){
		intakeZeroed = false;
		integral = 0;
	}

	void intakeRotationSet(double num){
		if(num < 0 && !topLimit.get())
			num = 0;
		intakeUp0.set(kPercentOutput,-num);
		intakeUp1.set(kPercentOutput,-num);
	}

	void controllerMove(Joystick buttonBoard) {
		SmartDashboard.putNumber("intake enc raw", encoder.getRaw());
		SmartDashboard.putBoolean("intake top limit", topLimit.get());
		//Sucking the ball
		if (buttonBoard.getRawButton(BUTTONS.INTAKE_IN)){
			intake.set(kPercentOutput, -1.0); 
		} else if (buttonBoard.getRawButton(BUTTONS.INTAKE_OUT)){
			intake.set(kPercentOutput, 1.0); 
		} else {
			intake.set(kPercentOutput, 0);
		}
		// Moving the intake up and down
		int level = buttonBoard.getPOV(1) / 45;

		// First things first, let's zero the intake rotation
		// intakeZeroed = true;
		if(topLimit.get() && !intakeZeroed) {
			intakeRotationSet(INTAKE_ROTATION_MAX);
		} else {
			if(!intakeZeroed)
				encoder.reset();
				intakeZeroed = true;
			switch (level){
				case 0: { // Start
					//intakePID(INTAKE_HEIGHTS.START); break;
					if (topLimit.get()) intakeZeroed = false;
					else intakeRotationSet(0);
					break;
				}
				case 1: // Ball
					intakePID(INTAKE_HEIGHTS.BALL); break;
				case 2: // Level
					intakePID(INTAKE_HEIGHTS.LEVEL); break;
				case 3: { // Override
					double intake_speed = 0.7;
					if (buttonBoard.getRawButton(BUTTONS.INTAKE_DEFAULT)) intake_speed = -buttonBoard.getRawAxis(BUTTONS.INTAKE_SPEED) / 2.0 + 0.5;
					if (buttonBoard.getRawButton(BUTTONS.INTAKE_UP) /* && encoder.getRaw() < INTAKE_HEIGHTS.LEVEL */){
						intakeRotationSet(intake_speed);
					} else if (buttonBoard.getRawButton(BUTTONS.INTAKE_DOWN) && topLimit.get()) {
						intakeRotationSet(-intake_speed);
					} else {
						intakeRotationSet(0);
					}
					break;}
				default: // Unknown
					intakeRotationSet(0);
			}
		}
	}

	boolean intakePID(double goal) {
		if (firstTime) {
			integral = 0;
			prevError = 0;
			firstTime = false;
		}
		double error = goal - encoder.getRaw();
		if (Math.abs(error) < TOLERANCE) {
			// End the function right here
			firstTime = true;
			integral = 0;
			intakeRotationSet(0.0);
			return true;
		}

		if (Math.abs(error) > PID_RANGE) {
			intakeRotationSet(error > 0 ? 0.8 : -0.8);
			return false;
		}

		SmartDashboard.putNumber("intake PID error", error);

		error /= 800000;

		integral += error;

		double derivative = error - prevError;

		double speed = error * INTAKE_KP + integral * INTAKE_KI + derivative * INTAKE_KD;
		prevError = error;
		if (speed > 0.8) speed = 0.8;
		if (speed < -0.8) speed = -0.8;
		// if (speed < 0 && speed > -0.25) speed = -0.25;
		SmartDashboard.putNumber("intake PID speed", speed);
		intakeRotationSet(speed);
		return false;
	}
}

class INTAKE_HEIGHTS {
	public final static int START = 0,
	BALL = 651877,
	LEVEL = 859670;
};