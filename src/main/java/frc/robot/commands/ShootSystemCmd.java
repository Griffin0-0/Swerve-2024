package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShootSystemCmd extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private final ShooterSubsystem shooterSubsystem;

    
    public Command sendShoot() {
        return Commands.startEnd(() -> shoot(), () -> stop());
    }

    
    public ShootSystemCmd(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {

        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(intakeSubsystem, shooterSubsystem);
    }

    public void shoot() {
        if (shooterSubsystem.getRollerSpeed() > 0.9) {
            intakeSubsystem.runIntake(-IntakeConstants.kIntakeMotorSpeed);
        }
    }

    public void stop() {
        intakeSubsystem.stop();
        shooterSubsystem.stop();
    }
}
