package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.wpilibj.Compressor;


public class ClimberSubsystem extends SubsystemBase {
    
    private final DoubleSolenoid solenoid_1;
    private final DoubleSolenoid solenoid_2;

    public boolean extended = false;

    public final Compressor compressor;


    public ClimberSubsystem() {

        solenoid_1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);
        solenoid_2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 9, 8);
        solenoidReverse();

        compressor = new Compressor(1, PneumaticsModuleType.REVPH);
        compressor.enableAnalog(115, 120); 
        }

    public void toggleExtention() {
        if (extended) {
            solenoidReverse();
        } else {
            solenoidForward();
        }
    }

    public void solenoidForward() {
        solenoid_1.set(kForward);
        solenoid_2.set(kForward);
        extended = true;
    }

    public void solenoidReverse() {
        solenoid_1.set(kReverse);
        solenoid_2.set(kReverse);
        extended = false;
    }
}
