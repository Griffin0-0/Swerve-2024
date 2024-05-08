package frc.robot.subsystems;

import java.util.Map;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;


public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.FRONT_LEFT_DRIVE_MOTOR_PORT,
        DriveConstants.FRONT_LEFT_TURNING_MOTOR_PORT,
        DriveConstants.MODULE_DRIVE_ENCODER_REVERSED,
        DriveConstants.MODULE_TURNING_ENCODER_REVERSED,
        DriveConstants.FRONT_LEFT_CAN_CODER_ID,
        DriveConstants.FRONT_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.MODULE_CAN_CODER_REVERSED);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.FRONT_RIGHT_DRIVE_MOTOR_PORT,
        DriveConstants.FRONT_RIGHT_TURNING_MOTOR_PORT,
        DriveConstants.MODULE_DRIVE_ENCODER_REVERSED,
        DriveConstants.MODULE_TURNING_ENCODER_REVERSED,
        DriveConstants.FRONT_RIGHT_CAN_CODER_ID,
        DriveConstants.FRONT_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.MODULE_CAN_CODER_REVERSED);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.BACK_LEFT_DRIVE_MOTOR_PORT,
        DriveConstants.BACK_LEFT_TURNING_MOTOR_PORT,
        DriveConstants.MODULE_DRIVE_ENCODER_REVERSED,
        DriveConstants.MODULE_TURNING_ENCODER_REVERSED,
        DriveConstants.BACK_LEFT_CAN_CODER_ID,
        DriveConstants.BACK_LEFT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.MODULE_CAN_CODER_REVERSED);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.BACK_RIGHT_DRIVE_MOTOR_PORT,
        DriveConstants.BACK_RIGHT_TURNING_MOTOR_PORT,
        DriveConstants.MODULE_DRIVE_ENCODER_REVERSED,
        DriveConstants.MODULE_TURNING_ENCODER_REVERSED,
        DriveConstants.BACK_RIGHT_CAN_CODER_ID,
        DriveConstants.BACK_RIGHT_DRIVE_ABSOLUTE_ENCODER_OFFSET_RAD,
        DriveConstants.MODULE_CAN_CODER_REVERSED);

    public final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public final LimeLight limeLight = new LimeLight();
    public Pose2d pose;
    public Pose2d limeLightPose;
    public boolean isAllianceBlue;
    public int tick = 0;
    public double[] headingBuffer = new double[10];
    public Translation2d[] translationBuffer = new Translation2d[5];
    boolean goodHeadingBuffer = false;
    boolean goodTranslationBuffer = false;
    public boolean fieldOriented = false;

    private final GenericEntry sb_gyro, sb_voltage, sb_time, sb_coord;
    

    public SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};

    public final SwerveDriveOdometry kOdometry = new SwerveDriveOdometry(
            Constants.DriveConstants.DRIVE_KINEMATICS, getRotation2d(),
            new SwerveModulePosition[] {
              backLeft.getPosition(),
              backRight.getPosition(),
              frontLeft.getPosition(),
              frontRight.getPosition()
            }, new Pose2d(0, 0, new Rotation2d()));

    public SwerveSubsystem() {
        // UNCOMMENT THIS CODE FOR COMPETITION: v

        // if (DriverStation.getAlliance().get() == Alliance.Red) {
        //     this.isAllianceBlue = false;
        // } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
        //     this.isAllianceBlue = true;
        // }

        // ^

        // COMMENT THIS CODE FOR COMPETITION: v

        Alliance test = Alliance.Red;

        if (test == Alliance.Red) {
            this.isAllianceBlue = false;
        } else if (test == Alliance.Blue) {
            this.isAllianceBlue = true;
        }

        // ^

        this.tick = 0;

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.setAngleAdjustment(180);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();



        sb_gyro = Shuffleboard.getTab("Driver")
            .add("Gyro", 0.0)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 2)
            .withSize(3, 3)
            .getEntry();

        sb_voltage = Shuffleboard.getTab("Driver")
            .add("Voltage", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .withPosition(8, 0)
            .withSize(4, 3)
            .getEntry();

        sb_time = Shuffleboard.getTab("Driver")
            .add("Time", 0.0)
            .withWidget(BuiltInWidgets.kTextView)
            .withPosition(0, 0)
            .withSize(3, 1)
            .getEntry();
        
        sb_coord = Shuffleboard.getTab("Driver")
            .add("Coordinates", "")
            .withPosition(8, 3)
            .withSize(4, 1)
            .getEntry();
    }

    public Command zeroHeading() {
        System.out.println("Gyro Reset");
        return Commands.runOnce(() -> gyro.reset()); // Returns a command to be used on button press
    }

    public Command coordinate() {
        System.out.println("new button worked");
        return Commands.runOnce(() -> coordinateFunction());
    }

    public double getHeading() {
        return Math.IEEEremainder(DriveConstants.GYRO_REVERSED ? gyro.getAngle() * -1 : gyro.getAngle(), 360);
    }

    public Pose2d getPose() {
        return kOdometry.getPoseMeters();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public void resetOdometry(Pose2d pose) {
        kOdometry.resetPosition(getRotation2d(), new SwerveModulePosition[] {
            backLeft.getPosition(),
            backRight.getPosition(),
            frontLeft.getPosition(),
            frontRight.getPosition()
          }, pose);
    }

    @Override
    public void periodic() {
        try {
        tick++;

        //Get pose from limelight
        limeLightPose = limeLight.runLimeLight(isAllianceBlue);

        // If limelight sees targets, then update odometry translation
        if (limeLight.seesMultipleTargets() || limeLight.seesOneGoodTarget()) {

            int tickConstrainedTranslation = tick % 5;

            if (tickConstrainedTranslation == 0) {
                goodTranslationBuffer = true;
            }

            translationBuffer[tickConstrainedTranslation] = limeLightPose.getTranslation();

            if (tickConstrainedTranslation == 4 && goodTranslationBuffer) {
                double sumX = 0;
                double sumY = 0;

                for (int i = 0; i < 5; i++) {
                    sumX += translationBuffer[i].getX();
                    sumY += translationBuffer[i].getY();
                }

                new Translation2d();
                Translation2d averagedTranslation = new Translation2d(sumX / 5, sumY / 5);

                // Update odometry with limelight pose
                resetOdometry(new Pose2d(averagedTranslation, getRotation2d()));
            }
        } else {
            goodTranslationBuffer = false;
        }

        if (limeLight.seesMultipleTargets()) {

            // If limelight sees multiple targets, then update odometry rotation with an average of the last 10 headings
            int tickConstrainedHeading = tick % 10;

            if (tickConstrainedHeading == 0) {
                goodHeadingBuffer = true;
            }

            headingBuffer[tickConstrainedHeading] = limeLightPose.getRotation().getDegrees();

            

            if (tickConstrainedHeading == 9 && goodHeadingBuffer) {

                double sum = 0;

                for (int i = 0; i < 10; i++) {
                    sum += headingBuffer[i];
                }

                new Rotation2d();
                Rotation2d averagedHeading = Rotation2d.fromDegrees(sum / 10);
                // averagedHeading.minus(Rotation2d.fromDegrees(180));

                gyro.reset();

                // Field orient based on Limelight
                gyro.setAngleAdjustment(-averagedHeading.getDegrees());

                fieldOriented = true;
            }
        } else {
            goodHeadingBuffer = false;
        }

        // Update odometry with swerve module positions
        pose = kOdometry.update(getRotation2d(),
            new SwerveModulePosition[] {
                backLeft.getPosition(),
                backRight.getPosition(),
                frontLeft.getPosition(),
                frontRight.getPosition()
            });

        SmartDashboard.putString("Pose", pose.toString());

        sb_gyro.setDouble(getHeading() - 180);
        sb_voltage.setDouble(RobotController.getBatteryVoltage());
        sb_time.setDouble(DriverStation.getMatchTime());
        sb_coord.setString(getPose().toString());
        } catch (Exception ex) {
            ex.printStackTrace();
        }
    }
    
    public void coordinateFunction() {
        resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
        backLeft.resetEncoders();
        backRight.resetEncoders();
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.PHYSICAL_MAX_SPEED_METERS_PER_SEC);
        backLeft.setDesiredState(desiredStates[0]);
        backRight.setDesiredState(desiredStates[1]);
        frontLeft.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }
}
