package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.Compressor;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;


public class ClimberSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid solenoid_1;
    private final DoubleSolenoid solenoid_2;

    public final Compressor compressor;

    private final GenericEntry sb_pressure, devsb_pressure;


    public ClimberSubsystem() {

        solenoid_1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        solenoid_2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 9, 8);
        solenoidReverse();

        compressor = new Compressor(1, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(115, 120); 

        sb_pressure = Shuffleboard.getTab("Driver")
            .add("Compressor Pressure", 0.0)
            .withPosition(0, 1)
            .withSize(3, 1)
            .getEntry();

        devsb_pressure = Shuffleboard.getTab("Developer")
            .add("Compressor Pressure", 0.0)
            .getEntry();
        }

    public void solenoidForward() {
        solenoid_1.set(kForward);
        solenoid_2.set(kForward);
    }

    public void solenoidReverse() {
        solenoid_1.set(kReverse);
        solenoid_2.set(kReverse);
    }

    @Override
    public void periodic() {
        sb_pressure.setDouble(getPressure()); // Replace when running compressor
        devsb_pressure.setDouble(getPressure());
    }

    public double getPressure() {
        return 0; // compressor.getPressure();
    }
}
