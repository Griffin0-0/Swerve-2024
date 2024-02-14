package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimeLight;

public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontLeftCANCoderId,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kFrontRightCANCoderId,
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackLeftCANCoderId,
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final SwerveModule backRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kModuleDriveEncoderReversed,
        DriveConstants.kModuleTurningEncoderReversed,
        DriveConstants.kBackRightCANCoderId,
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kModuleCANCoderReversed);

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    public final LimeLight limeLight = new LimeLight();
    public Pose2d pose;
    public Pose2d limeLightPose;
    public boolean isAllianceBlue;
    public int tick = 0;
    public double[] headingBuffer = new double[10];
    boolean goodBuffer = false;
    boolean fieldOriented = false;
    

    public SwerveModule[] swerveModules = {frontLeft, frontRight, backLeft, backRight};

    public final SwerveDriveOdometry kOdometry = new SwerveDriveOdometry(
            Constants.DriveConstants.kDriveKinematics, getRotation2d(),
            new SwerveModulePosition[] {
              backLeft.getPosition(),
              backRight.getPosition(),
              frontLeft.getPosition(),
              frontRight.getPosition()
            }, new Pose2d(0, 0, new Rotation2d()));

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.setAngleAdjustment(180);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
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
        return Math.IEEEremainder(DriveConstants.kGyroReversed ? gyro.getAngle() * -1 : gyro.getAngle(), 360);
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
        tick++;
        SmartDashboard.putNumber("heading", getHeading());

        // UNCOMMENT THIS CODE FOR COMPETITION: v

        // if (DriverStation.getAlliance().get() == Alliance.Red) {
        //     SmartDashboard.putBoolean("Alliance:", false);
        //     isAllianceBlue = false;
        // } else if (DriverStation.getAlliance().get() == Alliance.Blue) {
        //     SmartDashboard.putBoolean("Alliance:", true);
        //     isAllianceBlue = true;
        // }

        // ^

        // COMMENT THIS CODE FOR COMPETITION: v

        Alliance test = Alliance.Blue;
        if (test == Alliance.Red) {
            SmartDashboard.putBoolean("Alliance:", false);
            isAllianceBlue = false;
        } else if (test == Alliance.Blue) {
            SmartDashboard.putBoolean("Alliance:", true);
            isAllianceBlue = true;
        }

        // ^

        limeLightPose = limeLight.runLimeLight(isAllianceBlue);

        if (limeLight.seesTargets()) {
            if (limeLight.seesMultipleTargets()) {
                int tickConstrained = tick % 10;

                if (tickConstrained == 0) {
                    goodBuffer = true;
                }

                headingBuffer[tickConstrained] = limeLightPose.getRotation().getDegrees();

                if (tickConstrained == 9 && goodBuffer) {
                    double sum = 0;

                    for (int i = 0; i < 10; i++) {
                        sum += headingBuffer[i];
                    }

                    new Rotation2d();
                    Rotation2d averagedHeading = Rotation2d.fromDegrees(sum / 10);

                    gyro.reset();
                    gyro.setAngleAdjustment(-averagedHeading.getDegrees());

                    fieldOriented = true;
                }
            } else {
                goodBuffer = false;
            }
            resetOdometry(new Pose2d(limeLightPose.getTranslation(), getRotation2d()));
        }

        SmartDashboard.putString("Limelight Pose", limeLightPose.toString());
        SmartDashboard.putBoolean("Limelight Sees Targets", limeLight.seesTargets());

        pose = kOdometry.update(getRotation2d(),
            new SwerveModulePosition[] {
                backLeft.getPosition(),
                backRight.getPosition(),
                frontLeft.getPosition(),
                frontRight.getPosition()
            });

        SmartDashboard.putNumber("X", pose.getX());
        SmartDashboard.putNumber("Y", pose.getY());
        SmartDashboard.putString("Pose", getPose().toString());
        SmartDashboard.putNumber("Odometry Rotation", pose.getRotation().getDegrees());
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
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        backLeft.setDesiredState(desiredStates[0]);
        backRight.setDesiredState(desiredStates[1]);
        frontLeft.setDesiredState(desiredStates[2]);
        frontRight.setDesiredState(desiredStates[3]);
    }
}
