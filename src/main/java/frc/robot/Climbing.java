package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Climbing {
	private final double EXTEND_TIME = 1.5, DISTANCE_TO_CONTACT = 40.0/39.37;
	private final double VELOCITY = (DISTANCE_TO_CONTACT/EXTEND_TIME);

	private DoubleSolenoid sol0, sol1;
	private Driving drv;
	private Timer timer;
	private AnalogInput ultra, pressureSensor;
	private boolean timing;
	private boolean timer_started;

    Climbing(Driving drive) {
        sol0 = new DoubleSolenoid(PORTS.CLIMB_SOL_PCM, PORTS.CLIMB_SOL0_0, PORTS.CLIMB_SOL0_1);
        sol1 = new DoubleSolenoid(PORTS.CLIMB_SOL_PCM, PORTS.CLIMB_SOL1_0, PORTS.CLIMB_SOL1_1);
        // sol1 = new DoubleSolenoid(1, 4, 5);
        //timer = new Timer();
        // Retract climbing pistons on the first run
        timer.stop();
        timer.reset();
        timing = true;
        ultra = new AnalogInput(PORTS.ULTRASONIC);
        pressureSensor = new AnalogInput(PORTS.PRESSURE_SENSOR);
        drv = drive;

        timer_started = false;
    }

    private void extend(boolean ext){
        if (ext){
            sol0.set(DoubleSolenoid.Value.kForward); // extend
            sol1.set(DoubleSolenoid.Value.kReverse);
        } else {
            sol0.set(DoubleSolenoid.Value.kReverse); // Retract
            sol1.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void pressureSense(){
        // Read pressure sensor
        double supply_voltage_normalized = 2.623290747 / (0.004 * 115 + 0.1);
        double voltage = pressureSensor.getVoltage();
        SmartDashboard.putNumber("Pressure Sensor Voltage", voltage);
        double pressure = 250.0 * voltage / supply_voltage_normalized - 25;
        // pressure = ((int)(pressure * 100)) / 100.0;
        SmartDashboard.putNumber("Pressure Sensor (PSI)", pressure);
        if (pressure > 60){
            SmartDashboard.putBoolean("Pressure Above 60 PSI", true);
        } else SmartDashboard.putBoolean("Pressure Above 60 PSI", false);
    }

    public void controllerMove(Joystick buttonBoard){
        if (buttonBoard.getPOV(0)/45 == 5){ // HAB 1 to 2 button
            SmartDashboard.putNumber("time elapsed", timer.get());
            SmartDashboard.putNumber("ULTRASONIC", getDistance());
            if(!timer_started) {
                timer.reset();
                timer.start();
                timer_started = true;
            }
            if(timer.get() > EXTEND_TIME) {
                drv.drive(0.0, 0.0, false);
                extend(false);
            }
            else {  
                drv.velocityPID(VELOCITY,0.0);
                extend(true);
            }
            // Check that we are not extended past 7 inches
            //if (getDistance() >= 7) extend(false);
            //else extend(true);

        }
        /*else if (buttonBoard.getPOV(0)/45 == 6){ // HAB 2 to 3 button

        }*/
        else if (buttonBoard.getRawButton(BUTTONS.CLIMB_EXTEND)){ // Button is being pressed
            extend(true);       
        }
        else { // Normal teleop, probably won't be used
            extend(false);
            if (timer_started){
                timer.stop();
                timer_started = false;
            }
        }

    }

    private double getDistance(){
        double voltage_source = 5.0;
        double scaling = voltage_source / 1024.0; // Volts per 5 mm
        double dist = 5 * (ultra.getValue() / scaling); // Distance in mm
        return dist * 0.0393701; // Distance in inches
    }
}