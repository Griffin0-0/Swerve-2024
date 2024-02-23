// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ToggleClimberCmd extends Command {

  ClimberSubsystem climberSubsystem;
  Boolean toExtend;

  public ToggleClimberCmd(ClimberSubsystem climberSubsystem, boolean toExtend) {
    this.climberSubsystem = climberSubsystem;
    this.toExtend = toExtend;
    addRequirements(climberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (toExtend) {
      climberSubsystem.solenoidForward();
    } else {
      climberSubsystem.solenoidReverse();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
