package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.IntakeConstants;



public class IntakeArticulate extends SubsystemBase {

    private final CANSparkMax articulateMotor;
    private final SparkPIDController articulatePID;
    private final RelativeEncoder articulateEncoder;
    private final GenericEntry trapezoid;

    public boolean intakeOut = false;

    public Command sendToggleArticulate() {
        return Commands.runOnce(() -> toggleArticulate());
    }

    public IntakeArticulate() {
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
        articulatePID.setReference(-31, ControlType.kPosition);
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


    @Override
    public void periodic() {
        trapezoid.setDouble(articulateEncoder.getPosition());
    }

    
}

// public class IntakeArticulate extends TrapezoidProfileSubsystem {
    

//     private final CANSparkMax articulateMotor;
//     private final SparkPIDController articulatePID;
//     private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 5, 5);

//     private final GenericEntry trapezoid;


//     public Command sendToggleArticulate(double value) {
//         return Commands.runOnce(() -> setGoal(value), this);
//     }


//     public IntakeArticulate() {
//         super(
//             new TrapezoidProfile.Constraints(10, 5), 0
//         );
//         articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
//         articulatePID = articulateMotor.getPIDController();

//         trapezoid = Shuffleboard.getTab("Driver")
//             .add("Trapezoid", 0.0)
//             .getEntry();
//       }

//     @Override
//     public void useState(TrapezoidProfile.State setpoint) {
//         // Calculate the feedforward from the sepoint
//         double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
//         // Add the feedforward to the PID output to get the motor output 

//         trapezoid.setDouble(setpoint.position);
//         articulatePID.setReference(setpoint.velocity, ControlType.kPosition, 0, feedforward);
//     }

//     public void isFinished() {
//         System.out.println("Done");
//         this.disable();
//     }
// }
