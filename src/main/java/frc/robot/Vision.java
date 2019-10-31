package frc.robot;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Vision {
	private Servo swivel, lift;

	Vision(){
		swivel = new Servo(PORTS.SWIVEL);
		lift = new Servo(PORTS.LIDAR_LIFT);
	}

	void swivel(Joystick buttonBoard){
		if (buttonBoard.getRawButton(BUTTONS.ASSIST_LITE)){
			swivel.set(88.5 / 180.0);
			return;
		}
		double angle = SmartDashboard.getNumber("Angle of Line", 0);
		double corrected = 95.5 - angle;
		swivel.set(corrected / 180.0); //normalize
	}

	void lift(Joystick buttonBoard){
		int level = buttonBoard.getPOV(0) / 45;
		if (buttonBoard.getRawButton(BUTTONS.LIDAR_UP)) {
			lift.set(0.225);
		}
		else if (buttonBoard.getRawButton(BUTTONS.LIDAR_DOWN)) {
			lift.set(0.4);
		}
		else {
			if(level > 0 && level % 2 == 0) lift.set(0.36); // Even level means cargo
			else if(level % 2 == 1) lift.set(0); // Odd level means hatch
		}
	}
}