// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;



public class ToggleArticulateCmd extends Command {

  IntakeSubsystem intakeSubsystem;
  public boolean intakeOut = false;

  public ToggleArticulateCmd(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (intakeOut) {
      intakeSubsystem.intakeUp();
      intakeOut = false;
    } else if (!intakeOut) {
      intakeSubsystem.intakeDown();
      intakeOut = true;
    }
  }

  @Override
  public void execute() {
    if (intakeOut && (intakeSubsystem.getPosition() < -30)) {
      intakeSubsystem.spinIn();
    } else {
      intakeSubsystem.stopIntake();
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
