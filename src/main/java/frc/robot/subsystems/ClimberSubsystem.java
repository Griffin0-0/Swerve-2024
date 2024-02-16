package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;


public class ClimberSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid solenoid_1;
    private final DoubleSolenoid solenoid_2;

    public boolean extended = false;

    // private final Compressor compressor;


    public Command sendToggleExtention() {
        return Commands.runOnce(() -> toggleExtention());
    }

    public ClimberSubsystem() {

        solenoid_1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        solenoid_2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 9, 8);

        // compressor = new Compressor(1, PneumaticsModuleType.REVPH);
        // compressor.enableAnalog(115, 120);
        }

    public void toggleExtention() {
        if (extended) {
            solenoidReverse();
            extended = false;

        } else if (!extended) {
            solenoidForward();
            extended = true;
        }
    }

    public void solenoidForward() {
        solenoid_1.set(kForward);
        solenoid_2.set(kForward);
    }

    public void solenoidReverse() {
        solenoid_1.set(kReverse);
        solenoid_2.set(kReverse);
    }
}
