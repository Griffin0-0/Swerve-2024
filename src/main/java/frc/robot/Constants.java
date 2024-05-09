package frc.robot;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    
    public static final class ModuleConstants {
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double DRIVE_MOTOR_GEAR_RATIO = 8.14 / 1.0; // Drive ratio of 8.14 : 1
        public static final double TURNING_MOTOR_GEAR_RATIO = 1.0 / (150.0 / 7.0); // Turning ratio of (150 / 7) : 1
        public static final double DRIVE_ENCODER_ROT_TO_METERS = DRIVE_MOTOR_GEAR_RATIO * Math.PI * WHEEL_DIAMETER_METERS;
        public static final double TURNING_ENCODER_ROT_TO_RAD = TURNING_MOTOR_GEAR_RATIO * 2 * Math.PI;
        public static final double DRIVE_ENCODER_RPM_TO_METERS_PER_SEC = DRIVE_ENCODER_ROT_TO_METERS / 60.0;
        public static final double TURNING_ENCODER_RPM_TO_RAD_PER_SEC = TURNING_ENCODER_ROT_TO_RAD / 60.0;
        public static final double P_TURNING = 0.5; // For PID
    }

    public static final class DriveConstants {
        /**Distance between the right and left wheels in meters*/
        public static final double TRACK_WIDTH = Units.inchesToMeters(29.5);
        /**Distance between the front and back wheels in meters*/
        public static final double WHEEL_BASE = Units.inchesToMeters(29.5);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2));

        // DRIVE Motor Ports
        public static final int FRONT_LEFT_DRIVE_MOTOR_PORT = 10;
        public static final int BACK_LEFT_DRIVE_MOTOR_PORT = 3;
        public static final int FRONT_RIGHT_DRIVE_MOTOR_PORT = 6;
        public static final int BACK_RIGHT_DRIVE_MOTOR_PORT = 8;

        // TURNING Motor Ports
        public static final int FRONT_LEFT_TURNING_MOTOR_PORT = 2;
        public static final int BACK_LEFT_TURNING_MOTOR_PORT = 4;
        public static final int FRONT_RIGHT_TURNING_MOTOR_PORT = 5;
        public static final int BACK_RIGHT_TURNING_MOTOR_PORT = 7;

        // CANCoder Ids
        public static final int FRONT_LEFT_CAN_CODER_ID = 22;
        public static final int BACK_LEFT_CAN_CODER_ID = 24;
        public static final int FRONT_RIGHT_CAN_CODER_ID = 23;
        public static final int BACK_RIGHT_CAN_CODER_ID = 21;

        // Invert booleans | We use MK4i modules so the turning motors are inverted
        public static final boolean MODULE_TURNING_ENCODER_REVERSED = true;
        public static final boolean MODULE_DRIVE_ENCODER_REVERSED = false;
        public static final boolean MODULE_CAN_CODER_REVERSED = false;
        public static final boolean GYRO_REVERSED = true;

        // Turning encoder offsets
        // public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -2.6;
        // public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 3.14159;
        // public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.436332;
        // public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 1.53589;

        public static final double FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = -148 * Math.PI / 180;
        public static final double BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = 180 * Math.PI / 180;
        public static final double FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = 25 * Math.PI / 180;
        public static final double BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD = 88 * Math.PI / 180;

        // Robot max speeds
        /**Physical max speed of the modules (safety limit)*/
        public static final double PHYSICAL_MAX_SPEED_METERS_PER_SEC = 3.6;
        /**Max speed set for normal teleop*/
        public static final double TELEDRIVE_MAX_SPEED_METERS_PER_SEC = 2.7;
        /**Max speed set for boosted teleop*/
        public static final double TELEBOOST_DRIVE_MAX_SPEED_METERS_PER_SEC = 3.2;
        /**Max speed set for slow teleop*/
        public static final double TELESLOW_DRIVE_MAX_SPEED_METERS_PER_SEC = 1.25;

        // Robot turning speeds
        public static final double PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_SEC = 2 * 2 * Math.PI;
        public static final double TELEDRIVE_MAX_ANGULAR_SPEED_RAD_PER_SEC = PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_SEC / 3;

        // Robot acceleration
        public static final double TELEDRIVE_MAX_ACCELERATION_UNITS_PER_SEC = 4;
        public static final double TELEDRIVE_MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC = 4;
    }

    public static final class OIConstants {
        // Ports
        public static final int OPERATOR_CONTROLLER_PORT = 0;
        public static final int DRIVER_TRANSLATE_STICK_PORT = 1;
        public static final int DRIVER_ROTATE_STICK_PORT = 2;
        public static final double DEADBAND = 0.1;

        // Joysticks
        public static final int DRIVER_X_AXIS = 0;
        public static final int DRIVER_Y_AXIS = 1;
        public static final int DRIVER_ROT_AXIS = 2;

        // Joystick Triggers
        public static final int DRIVER_BOOST_BUTTON_ID = 1;

        // Joystick Hatswitches
        public static final int DRIVER_SNAPBACK_BUTTON_ID = 15;

        // Buttons
        public static final int DRIVER_CLIMBER_DOWN_BUTTON_ID = 1; // X button
        public static final int DRIVER_RESET_GYRO_BUTTON_ID = 2; // B button
        public static final int DRIVER_TOGGLE_FLAP_BUTTON_ID = 3; //  button
        public static final int DRIVER_CLIMBER_UP_BUTTON_ID = 4; // Y button
        public static final int DRIVER_STOP_BUTTON_ID = 10; // "Start" button

        // POV
        public static final int DRIVER_UP_POV_ID = 0;
        public static final int DRIVER_RIGHT_POV_ID = 90;
        public static final int DRIVER_DOWN_POV_ID = 180;
        public static final int DRIVER_LEFT_POV_ID = 270;

        // Triggers
        public static final int DRIVER_SOURCE_INTAKE_TRIGGER_ID = 5; // Left bumper
        public static final int DRIVER_TOGGLE_INTAKE_TRIGGER_ID = 6; // Right bumper
        public static final int DRIVER_AMP_OUT_TRIGGER_ID = 7; // Left trigger
        public static final int DRIVER_SHOOT_TRIGGER_ID = 8; // Right trigger
    } 

    public static final class ShooterConstants {
        public static final int SPIN_MOTOR_1_ID = 50;
        public static final int SPIN_MOTOR_2_ID = 52;
        public static final int FLAP_SERVO_1_ID = 9;
        public static final int FLAP_SERVO_2_ID = 8;

        public static final double FLYWHEEL_SPEED = 1.0;
        public static final double INTAKE_SPEED = 0.25;
        public static final double AMP_SPEED = 0.15;

        public static final double SPEED_CAP = 1.0;

        public static final double FLAP_SPEAKER_POS = 65;
        public static final double FLAP_DEFAULT_POS = 0;
        public static final double FLAP_AMP_POS = 40;
    }

    public static final class IntakeConstants {
        public static final int MOTOR_ID = 51;
        public static final int ARTICULATE_MOTOR_ID = 54;

        public static final double GROUND_MOTOR_SPEED = 0.7;
        public static final double SOURCE_MOTOR_SPEED = 0.4;
        public static final double IN_MOTOR_SPEED = 0.25;
        public static final double OUT_MOTOR_SPEED = -0.5;
        public static final double AMP_MOTOR_SPEED = -0.72;

        public static final double ARTICULATE_SPEED = 0.4;
        public static final double ARTICULATE_ACCELERATION_UNITS_PER_SEC = 3;

        public static final double AMP_DESIRED_POS = -20.5;
        public static final double STORE_DESIRED_POS = 0;
        public static final double OUT_DESIRED_POS = -44.5;
    }

    public static final class AutoConstants {

        //Speed
        /**Max speed set for auto*/
        public static final double MAX_SPEED_METERS_PER_SEC = 1.25;
        public static final double GROUND_INTAKE_MAX_SPEED_METERS_PER_SEC = 1.25;

        //Accel
        public static final double MAX_ACCELERATION_UNITS_PER_SEC = 6;
        public static final double MAX_ANGULAR_ACCELERATION_UNITS_PER_SEC = 4;

        //Turning speed
        public static final double MAX_ANGULAR_SPEED_RAD_PER_SEC = DriveConstants.PHYSICAL_MAX_ANGULAR_SPEED_RAD_PER_SEC / 4;

        //Min Speed
        public static final double MIN_SPEED = 0.02;
        public static final double MIN_TURN_SPEED_RAD = 0.05;
    
        //Tick things
        public static final int SPEAKER_SHOT_CHECK_TICKS = 50;
        public static final int CLOSE_SPEAKER_SHOT_CHECK_TICKS = 400;
        public static final int GROUND_INTAKE_CHECK_TICKS = 7;
        public static final int DEPOSIT_CHECK_TICKS = 300;
        public static final int SOURCE_COLOR_CHECK_TICKS = 100;

        // Tolerances
        public static final double TOLERANCE_METERS = 0.09;
        public static final double TOLERANCE_DEGREES = 5.0;
    }
}