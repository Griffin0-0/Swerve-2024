package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;

    private final DigitalInput limitSwitch_1;
    private final DigitalInput limitSwitch_2;
    private final DigitalInput limitSwitch_3;

    private final CANSparkMax articulateMotor;
    private final SparkPIDController articulatePID;
    private final RelativeEncoder articulateEncoder;
    private final GenericEntry trapezoid;

    public boolean intakeOut = false;


    public Command sendToggleArticulate() {
        return Commands.runOnce(() -> toggleArticulate());
    }

    public Command sendStop() {
        return Commands.runOnce(() -> stop());
    }

    public Command sendIntakeSpinIn() {
        return Commands.runOnce(() -> runIntake(IntakeConstants.kIntakeMotorSpeed));
    }

    public Command sendIntakeSpinOut() {
        return Commands.runOnce(() -> runIntake(-IntakeConstants.kIntakeMotorSpeed));
    }


    public IntakeSubsystem() {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

        limitSwitch_1 = new DigitalInput(0);
        limitSwitch_2 = new DigitalInput(1);
        limitSwitch_3 = new DigitalInput(2);

        articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
        articulatePID = articulateMotor.getPIDController();
        articulateEncoder = articulateMotor.getEncoder();

        articulatePID.setOutputRange(-0.15, 0.15);
        articulatePID.setP(0.5);
        articulatePID.setI(0);
        articulatePID.setD(0);

        trapezoid = Shuffleboard.getTab("Driver")
            .add("Trapezoid", 0.0)
            .withWidget(BuiltInWidgets.kGraph)
            .getEntry();
    }

    public void intakeDown() {
        articulatePID.setReference(-32, ControlType.kPosition);
    }
    public void intakeUp() {
        articulatePID.setReference(-0.5, ControlType.kPosition);
    }
    
    public void toggleArticulate() {
        if (intakeOut) {
            intakeUp();
            intakeOut = false;
        } else if (!intakeOut) {
            intakeDown();
            intakeOut = true;
        }
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    @Override
    public void periodic() {
        if (intakeOut && (articulateEncoder.getPosition() < -30)) {
            runIntake(IntakeConstants.kIntakeMotorSpeed);
        } else {
            runIntake(0);
        }
    }

    public void stop() {
        intakeMotor.set(0);
        articulateMotor.set(0);
    }
}
