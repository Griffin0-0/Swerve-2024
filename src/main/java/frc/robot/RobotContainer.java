// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.math.geometry.Pose2d; // For MoveToPosCmd
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.auto.DepositToAmpCmd;
import frc.robot.commands.auto.SimpleFireAtSpeakerCmd;
import frc.robot.commands.auto.SimpleIntakeFromGroundCmd;
import frc.robot.commands.functions.EmergencyStopMechanismsCmd;
import frc.robot.commands.functions.ShootAmpCmd;
import frc.robot.commands.functions.ShootCmd;
import frc.robot.commands.functions.SourceIntakeCmd;
import frc.robot.commands.functions.ToggleArticulateCmd;
import frc.robot.commands.functions.ToggleClimberCmd;
import frc.robot.commands.functions.ToggleFlapCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ClimberSubsystem;


public class RobotContainer {

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem ledSubsystem = new LEDSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(ledSubsystem);

  private final Joystick driverJoystick = new Joystick(OIConstants.kOperatorControllerPort);
  private final Joystick translateStick = new Joystick(OIConstants.kDriverTranslateStickPort);
  private final Joystick rotateStick = new Joystick(OIConstants.kDriverRotateStickPort);

  UsbCamera intakeCamera;

  public RobotContainer() {

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -translateStick.getRawAxis(OIConstants.kDriverYAxis),
      () -> -translateStick.getRawAxis(OIConstants.kDriverXAxis),
      () -> rotateStick.getRawAxis(0),
      () -> translateStick.getRawButton(2),
      () -> translateStick.getRawButton(1), // Trigger
      () -> rotateStick.getRawButton(1))); // Trigger

    // swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
    //             swerveSubsystem,
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverYAxis),
    //             () -> -driverJoystick.getRawAxis(OIConstants.kDriverXAxis),
    //             () -> driverJoystick.getRawAxis(OIConstants.kDriverRotAxis),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverCoordinateButtonId),
    //             () -> driverJoystick.getRawButton(OIConstants.kDriverShootButtonId),
    //             () -> !driverJoystick.getRawButton(OIConstants.kDriverFieldOrientedButtonId)));

    intakeCamera = CameraServer.startAutomaticCapture(0);
                
    configureBindings();
  }

  private void configureBindings() {
    new JoystickButton(driverJoystick, OIConstants.kDriverResetGyroButtonId).onTrue(swerveSubsystem.zeroHeading());
    new JoystickButton(driverJoystick, OIConstants.kDriverFireSpeakerButtonId).whileTrue(new SequentialCommandGroup(new SimpleFireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem)));
    
    new JoystickButton(driverJoystick, OIConstants.kDriverStopButtonId).onTrue(new EmergencyStopMechanismsCmd(shooterSubsystem, intakeSubsystem, climberSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleClimberButtonId).onTrue(new ToggleClimberCmd(climberSubsystem, true));
    new JoystickButton(driverJoystick, 9).onTrue(new ToggleClimberCmd(climberSubsystem, false));
    new JoystickButton(driverJoystick, OIConstants.kDriverSourceIntakeButtonId).whileTrue(new SourceIntakeCmd(shooterSubsystem, intakeSubsystem));

    new POVButton(driverJoystick, OIConstants.kDriverAmpDepositId).whileTrue(new DepositToAmpCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem));


    // TRIGGERS
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleFlapButtonId).whileTrue(new ToggleFlapCmd(shooterSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverToggleGroundIntakeButtonId).whileTrue(new ToggleArticulateCmd(intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverRunAmpButtonId).whileTrue(new ShootAmpCmd(shooterSubsystem, intakeSubsystem));
    new JoystickButton(driverJoystick, OIConstants.kDriverRunShooterButtonId).whileTrue(new ShootCmd(shooterSubsystem, intakeSubsystem, ledSubsystem));
  }



  public Command getAutonomousCommand() {
    // Pose2d[] path1 = { // To create a path for MoveToPosCmd
    // };

    return new SequentialCommandGroup(
          new SimpleFireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem),
          new SimpleIntakeFromGroundCmd(swerveSubsystem, intakeSubsystem, new Translation2d(3.5, 5.45)),
          new SimpleFireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem),
          new SimpleIntakeFromGroundCmd(swerveSubsystem, intakeSubsystem, new Translation2d(3.5, 6.85)),
          new SimpleFireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem),
          new SimpleIntakeFromGroundCmd(swerveSubsystem, intakeSubsystem, new Translation2d(3.5, 4.05)),
          new SimpleFireAtSpeakerCmd(swerveSubsystem, shooterSubsystem, intakeSubsystem),
          new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}

//Import waitCommand for waiting
