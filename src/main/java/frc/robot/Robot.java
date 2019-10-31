/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
	private final SendableChooser<String> m_chooser = new SendableChooser<>();
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;

	private Joystick leftJoy = new Joystick(1),
	rightJoy = new Joystick(2),
	buttonBoard = new Joystick(3);

	// Helper classes
	private Driving drv;
	private Arm arm;
	private Vision viz;
	private Intake intake;
	private Climbing climb;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
		// m_chooser.addOption("My Auto", kCustomAuto);
		// SmartDashboard.putData("Auto choices", m_chooser);
		m_chooser.addOption("Yes zero", "zero");
		m_chooser.setDefaultOption("No zero", "notzero");
		SmartDashboard.putData("Zeroing at Init", m_chooser);

		// Joystick inputs
		leftJoy = new Joystick(1);
		rightJoy = new Joystick(2);
		buttonBoard = new Joystick(3);

		// Helper classes
		drv = new Driving();
		arm = new Arm();
		intake = new Intake();
		climb = new Climbing(drv);
		// viz = new Vision();
	}

	/**
	 * Periodic code for all robot modes should go here.
	 */
	@Override
	public void robotPeriodic() {
		climb.pressureSense();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode.
	 * 
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable chooser
	 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
	 * remove all of the chooser code and uncomment the GetString line to get the
	 * auto name from the text box below the Gyro.
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the SendableChooser
	 * make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		drv.reset();
		if (m_chooser.getSelected().compareTo("zero") == 0){
			intake.reset();
			arm.reset();
		} else {
			intake.intakeZeroed = true;
			arm.armZeroed = true;
			arm.liftZeroed = true;
		}
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		teleopPeriodic();
		if (m_autoSelected == kCustomAuto) {
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}
	}

	/**
	 * This function is called once each time the robot enters teleoperated mode.
	 */
	@Override
	public void teleopInit() {
		/* drv.reset();
		if (m_chooser.getSelected().compare("zero") == 0){
			intake.reset();
			arm.reset();
		} else {
			intake.intakeZeroed = true;
			arm.armZeroed = true;
			arm.liftZeroed = true;
		} */
	}

	/**
	 * This function is called periodically during teleoperated mode.
	 */
	@Override
	public void teleopPeriodic() {
		// Driving
		// drv.VelocityPID(1,0.0);
		drv.controllerMove(leftJoy, rightJoy, buttonBoard);
		// drv.AutoShift();
		// Arm
		arm.controllerMove(rightJoy, buttonBoard);
		// Intake
		intake.controllerMove(buttonBoard);
		// Vision
		viz.swivel(buttonBoard);

		// viz.Lift(buttonBoard);
		// Climbing
		climb.controllerMove(buttonBoard);
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {}
}
