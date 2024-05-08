// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class SourceIntakeCmd extends Command {

  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  private int tickCheck = AutoConstants.SOURCE_COLOR_CHECK_TICKS;

  public SourceIntakeCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.spinIn();
    intakeSubsystem.spinIn();
    shooterSubsystem.flapAmp();
    tickCheck = AutoConstants.SOURCE_COLOR_CHECK_TICKS;
  }

  @Override
  public void execute() {
    tickCheck--;
    if (intakeSubsystem.noteConfirmed && tickCheck <= 0) {
      intakeSubsystem.stopIntake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shooterStop();
    intakeSubsystem.stopIntake();
    shooterSubsystem.flapDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
