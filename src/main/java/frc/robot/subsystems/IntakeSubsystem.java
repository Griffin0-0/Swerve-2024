package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final CANSparkMax articulateMotor;
    private final SparkPIDController articulatePID;
    private final RelativeEncoder articulateEncoder;
    private final GenericEntry trapezoid, isOut;

    public boolean intakeOut = false;
    public boolean switchTemp = false;


    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

        articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
        articulatePID = articulateMotor.getPIDController();
        articulateEncoder = articulateMotor.getEncoder();
        articulateEncoder.setPosition(0);

        articulatePID.setOutputRange(-IntakeConstants.kIntakeArticulateSpeed, IntakeConstants.kIntakeArticulateSpeed);
        articulatePID.setP(0.6);
        articulatePID.setI(0);
        articulatePID.setD(0);

        trapezoid = Shuffleboard.getTab("Driver")
            .add("Trapezoid", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();

        isOut = Shuffleboard.getTab("Driver")
        .add("Is Out", false)
        .getEntry();

        articulateEncoder.setPosition(0);
    }

    public void intakeDown() {
        articulatePID.setReference(-34, ControlType.kPosition);
    }
    public void intakeUp() {
        articulatePID.setReference(-0.5, ControlType.kPosition);
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void toggleIntake() {
        if (intakeOut) {
            intakeUp();
            intakeOut = false;
        } else if (!intakeOut) {
            intakeDown();
            intakeOut = true;
        }
    }

    public boolean isDown() {
        if (intakeOut) {
            if (getPosition() < -30) {
                return true;
            }
        }
        return false;
    }

    public void spinIn() {
        runIntake(IntakeConstants.kGroundIntakeMotorSpeed);
    }

    public void spinOut() {
        runIntake(-IntakeConstants.kIntakeOutMotorSpeed);
    }

    public double getPosition() {
        return articulateEncoder.getPosition();
    }

    @Override
    public void periodic() {
        trapezoid.setDouble(articulateEncoder.getPosition());
        isOut.setBoolean(intakeOut);

        if (isDown()) {
            spinIn();
            switchTemp = true;
        } else if (switchTemp) {
            stopIntake();
            switchTemp = false;
        }
    }

    public void stop() {
        intakeMotor.set(0);
        articulateMotor.set(0);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }
}
