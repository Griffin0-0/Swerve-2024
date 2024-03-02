// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.functions;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootCmd extends Command {

  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  LEDSubsystem ledSubsystem;
  Boolean isDone = false;
  int tickLimit = AutoConstants.kAutoCloseSpeakerShotCheckTick;

  public ShootCmd(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem, LEDSubsystem ledSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.ledSubsystem = ledSubsystem;
    addRequirements(shooterSubsystem, intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.speakerSpinOut();
    ledSubsystem.setShoot();
    shooterSubsystem.flapAmp();
    isDone = false;
    tickLimit = AutoConstants.kAutoCloseSpeakerShotCheckTick;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterSubsystem.getRollerSpeed() > 0.99) {
      intakeSubsystem.spinOut();
    }
    tickLimit--;
    if (tickLimit < 0) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.shooterStop();
    intakeSubsystem.stopIntake();
    ledSubsystem.setDefault();
    shooterSubsystem.flapDefault();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
