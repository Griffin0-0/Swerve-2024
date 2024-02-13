package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.IntakeConstants;

public class IntakeArticulate extends TrapezoidProfileSubsystem {
    

    private final CANSparkMax articulateMotor;
    private final SparkPIDController articulatePID;
    private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 3, 3);

    private final GenericEntry trapezoid;


    public Command sendToggleArticulate(double value) {
        return Commands.runOnce(() -> setGoal(value), this);
    }


    public IntakeArticulate() {
        super(
            new TrapezoidProfile.Constraints(0.3, 0.1), 0
        );
        articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
        articulatePID = articulateMotor.getPIDController();

        trapezoid = Shuffleboard.getTab("Driver")
            .add("Trapezoid", 0.0)
            .withWidget(BuiltInWidgets.kGraph) // specify the widget here
            .getEntry();
      }

    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
        // Add the feedforward to the PID output to get the motor output

        trapezoid.setDouble(setpoint.velocity);
        articulatePID.setReference(setpoint.velocity, ControlType.kVelocity, 0, feedforward);
    }
}
