// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.functions.EmergencyStopMechanismsCmd;
import frc.robot.commands.functions.ShootAmpCmd;
import frc.robot.commands.functions.ShootCmd;
import frc.robot.commands.functions.SourceIntakeCmd;
import frc.robot.commands.functions.ToggleArticulateCmd;
import frc.robot.commands.functions.ToggleClimberCmd;
import frc.robot.commands.functions.ToggleFlapCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ClimberSubsystem;


public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final SwerveSubsystem swerveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final ClimberSubsystem climberSubsystem;
  private final Limelight limelight;

  private final Joystick driverJoystick = new Joystick(OIConstants.kOperatorControllerPort);
  private final Joystick translateStick = new Joystick(OIConstants.kDriverTranslateStickPort);
  private final Joystick rotateStick = new Joystick(OIConstants.kDriverRotateStickPort);

  public RobotContainer() {

    climberSubsystem = new ClimberSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    shooterSubsystem = new ShooterSubsystem();
    limelight = new Limelight();
    swerveSubsystem = new SwerveSubsystem(limelight);

    NamedCommands.registerCommand("useIntake", new ToggleArticulateCmd(intakeSubsystem));
    NamedCommands.registerCommand("runShooter", new ShootCmd(shooterSubsystem, intakeSubsystem));

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -translateStick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -translateStick.getRawAxis(OIConstants.kDriverXAxis),
      () -> rotateStick.getRawAxis(0),
      () -> !driverJoystick.getRawButton(OIConstants.kDriverCoordinateButtonId),
      () -> translateStick.getRawButton(1),
      () -> rotateStick.getRawButton(1)));

    // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
    //             swerveSubsystem,
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
    //             () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverCoordinateButtonId),
    //             () -> driverJoystick.getRawButton(OIConstants.kDriverShootButtonId),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonId)));


    // NamedCommands.registerCommand("retractIntake", swerve.autoBalanceCommand());
    // NamedCommands.registerCommand("runShooter", swerve.autoBalanceCommand());
    // NamedCommands.registerCommand("stopShooter", swerve.autoBalanceCommand());

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser("Straight");
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonId).onTrue(swerveSubsystem.zeroHeading());
    // new JoystickButton(driverJoystick, OIConstants.kDriverCoordinateButtonId).onTrue(swerveSubsystem.coordinate());
    new JoystickButton(driverJoystick, OIConstants.kDriverStopButtonId).onTrue(new EmergencyStopMechanismsCmd(shooterSubsystem, intakeSubsystem, climberSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleClimberButtonId).onTrue(new ToggleClimberCmd(climberSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverSourceIntakeButtonId).whileTrue(new SourceIntakeCmd(shooterSubsystem, intakeSubsystem));

    // TRIGGERS
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleFlapButtonId).whileTrue(new ToggleFlapCmd(shooterSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleGroundIntakeButtonId).whileTrue(new ToggleArticulateCmd(intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverRunAmpButtonId).whileTrue(new ShootAmpCmd(shooterSubsystem, intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverRunShooterButtonId).whileTrue(new ShootCmd(shooterSubsystem, intakeSubsystem));
  }



  public Command getAutonomousCommand() {
    return new PathPlannerAuto("2 Note Auto");
  }
}
