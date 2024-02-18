package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public boolean checkLimits() {
        if (!limitSwitch_1.get() || !limitSwitch_2.get() || !limitSwitch_3.get()) {
            return true;
        } else {
            return false;
        }
    }

    public void stop() {
        intakeMotor.set(0);
    }
}
