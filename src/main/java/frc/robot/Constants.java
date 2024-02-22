package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
        public static final double kDriveMotorGearRatio = 8.14 / 1.0; // Drive ratio of 8.14 : 1
        public static final double kTurningMotorGearRatio = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60.0;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60.0;
        public static final double kPTurning = 0.5; // For PID
    }

    public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(29.5); // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(29.5); // Distance between front and back wheels

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        // DRIVE Motor Ports
        public static final int kFrontLeftDriveMotorPort = 10;
        public static final int kBackLeftDriveMotorPort = 3;
        public static final int kFrontRightDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 8;

        // TURNING Motor Ports
        public static final int kFrontLeftTurningMotorPort = 2;
        public static final int kBackLeftTurningMotorPort = 4;
        public static final int kFrontRightTurningMotorPort = 5;
        public static final int kBackRightTurningMotorPort = 7;

        // CANCoder Ids
        public static final int kFrontLeftCANCoderId = 22;
        public static final int kBackLeftCANCoderId = 24;
        public static final int kFrontRightCANCoderId = 23;
        public static final int kBackRightCANCoderId = 21;

        // Invert booleans | We use MK4i modules so the turning motors are inverted
        public static final boolean kModuleTurningEncoderReversed = true;
        public static final boolean kModuleDriveEncoderReversed = false;
        public static final boolean kModuleCANCoderReversed = false;
        public static final boolean kGyroReversed = true;

        // Turning encoder offsets
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -2.6;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.14159;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.436332;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.53589;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -148 * Math.PI / 180;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 180 * Math.PI / 180;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 25 * Math.PI / 180;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 88 * Math.PI / 180;

        // Robot speeds
        public static final double kPhysicalMaxSpeedMetersPerSecond = 3.6; // PHYSICAL max speed of the modules (safety cap) 3.6
        public static final double kTeleDriveMaxSpeedMetersPerSecond = 1; // Max speed set for teleop

        // Robot turning speeds
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        // Robot acceleration
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        // Robot speed modifiers
        public static final double kTeleopBoostModifier = 1.5;
        public static final double kTeleopSlowModifier = 0.5;
    }

    public static final class OIConstants {
        // Ports
        public static final int kOperatorControllerPort = 0;
        public static final int kDriverTranslateStickPort = 1;
        public static final int kDriverRotateStickPort = 2;
        public static final double kDeadband = 0.05;

        // Joysticks
        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 2;

        // Joystick Triggers
        public static final int kDriverBoostButtonId = 1;

        // Joystick Hatswitches
        public static final int kDriverSnapBackButtonId = 15;

        // Buttons
        public static final int kDriverResetGyroButtonId = 2;
        public static final int kDriverFireSpeakerButtonId = 3;
        public static final int kDriverToggleClimberButtonId = 4;
        public static final int kDriverStopButtonId = 10;
        public static final int kDriverSourceIntakeButtonId = 1;

        // Triggers
        public static final int kDriverToggleFlapButtonId = 5;
        public static final int kDriverToggleGroundIntakeButtonId = 6;
        public static final int kDriverRunAmpButtonId = 7;
        public static final int kDriverRunShooterButtonId = 8;
    } 

    public static final class ShooterConstants {
        public static final int kShooterSpinMotorId_1 = 50;
        public static final int kShooterSpinMotorId_2 = 52;

        public static final double kShooterFlywheelSpeed = 1.0;
        public static final double kShooterIntakeSpeed = 0.25;
        public static final double kShooterAmpSpeed = 0.15;

        public static final int kShooterFlapServoId_1 = 9;
        public static final int kShooterFlapServoId_2 = 8;
        public static final double kShooterFlapUpPos = 170;

        public static final double kShooterFlapAmpPos = 155;

        public static final double kShooterSpeedCap = 1.0;
    }

    public static final class IntakeConstants {
        public static final int kIntakeMotorId = 51;
        public static final double kGroundIntakeMotorSpeed = 0.7;
        public static final double kIntakeOutMotorSpeed = 0.5;

        public static final double kIntakeArticulateSpeed = 0.3;
        public static final int kIntakeArticulateMotorId = 54;
        public static final double kIntakeArticulateAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {

        //Speed
        public static final double kAutoMaxSpeedMetersPerSecond = 1.25; // Max speed set for auto
        public static final double kAutoGroundIntakingMaxSpeedMetersPerSecond = 1;

        //Accel
        public static final double kAutoMaxAccelerationUnitsPerSecond = 6;
        public static final double kAutoMaxAngularAccelerationUnitsPerSecond = 3;

        //Turning speed
        public static final double kAutoMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        //Min Speed
        public static final double kAutoMinSpeed = 0.02;
        public static final double kAutoMinTurnSpeedRadians = 0.05;
    
        public static final int kAutoAmpStoppedCheckTicks = 30; // Ticks it waits to make sure swerve has come to a complete stop before depositing note to amp 20

        public static final int kAutoSpeakerShotCheckTicks = 70;

        public static final int kAutoGroundIntakeCheckTicks = 100;

        // Tolerances
        public static final double kAutoToleranceMeters = 0.09;
        public static final double kAutoToleranceDegrees = 5.0;

        public static  final int kAutoDepositCheckTicks = 300;
    }
}