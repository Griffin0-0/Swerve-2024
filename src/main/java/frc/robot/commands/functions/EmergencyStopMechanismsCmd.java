// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class EmergencyStopMechanismsCmd extends Command {

  ShooterSubsystem shooterSubsystem;
  IntakeSubsystem intakeSubsystem;
  ClimberSubsystem climberSubsystem;

  public EmergencyStopMechanismsCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, ClimberSubsystem climberSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.climberSubsystem = climberSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem, climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // climberSubsystem.compressor.disable();
    shooterSubsystem.stop();
    intakeSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
