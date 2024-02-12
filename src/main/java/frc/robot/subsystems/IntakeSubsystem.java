package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final CANSparkMax articulateIntakeMotor;

    private final RelativeEncoder articulateEncoder;
    private final DigitalInput limitSwitch_1;
    private final DigitalInput limitSwitch_2;
    private final DigitalInput limitSwitch_3;

    private final SlewRateLimiter articulateLimiter;
    private final PIDController articulatePIDController;


    private final GenericEntry articulateIntakeEncoder, articulateDesiredPosition, articulateIntakePos;

    public String intakePos = "up";
    public double desiredPosition = 1; // currently 1 (up) to -31 (down)


    public Command sendToggleIntake() {
        return Commands.runOnce(() -> toggleIntake());
    }

    public Command sendStop() {
        return Commands.runOnce(() -> stop());
    }



    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);
        articulateIntakeMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);

        articulateLimiter = new SlewRateLimiter(IntakeConstants.kIntakeArticulateAccelerationUnitsPerSecond);
        articulateEncoder = articulateIntakeMotor.getEncoder();
        articulateEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);

        articulatePIDController = new PIDController(0.5 , 0, 0);
        articulatePIDController.enableContinuousInput(-Math.PI, Math.PI);

        limitSwitch_1 = new DigitalInput(0);
        limitSwitch_2 = new DigitalInput(1);
        limitSwitch_3 = new DigitalInput(2);


        articulateIntakeEncoder = Shuffleboard.getTab("Driver")
        .add("Articulate Intake Encoder", 0.0)
        .getEntry();

        articulateDesiredPosition = Shuffleboard.getTab("Driver")
        .add("Articulate Desired Position", 0.0)
        .getEntry();

        articulateIntakePos = Shuffleboard.getTab("Driver")
        .add("Articulate Intake Pos", "up")
        .getEntry();

    }

    

    public double getArticulatePosition() {
        return articulateEncoder.getPosition();
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }



    public void periodic() {


        articulateIntakeEncoder.setDouble(getArticulatePosition());
        articulateDesiredPosition.setDouble(desiredPosition);
        articulateIntakePos.setString(intakePos);

    }

    public boolean checkLimits() {
        if (!limitSwitch_1.get() || !limitSwitch_2.get() || !limitSwitch_3.get()) {
            return true;
        } else {
            return false;
        }
    }

    public void toggleIntake() {
        if (intakePos == "up") {
            intakePos = "down";
            runIntake(IntakeConstants.kIntakeMotorSpeed);
        } 

        else if (intakePos == "down") {
            intakePos = "up";
            runIntake(0);
        }
    }

    public void stop() {
        intakeMotor.set(0);
        articulateIntakeMotor.set(0);
    }

    public void stopArticulate() {
        articulateIntakeMotor.set(0);
    }
}
