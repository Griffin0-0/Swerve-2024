// package frc.robot.subsystems;

// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMax;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
// import frc.robot.Constants.IntakeConstants;

// public class IntakeArticulate extends TrapezoidProfileSubsystem {
    

//     private final CANSparkMax articulateMotor;
//     private final PIDController articulatePIDController;
//     private final ArmFeedforward m_feedforward = new ArmFeedforward(0, 0, 3, 3);

//     private final GenericEntry trapezoid;


//     public IntakeArticulate() {
//         super(
//             new TrapezoidProfile.Constraints(0.3, 0.1), 0
//         );
//         articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
//         articulatePIDController = new PIDController(0.5 , 0, 0);

//         trapezoid = Shuffleboard.getTab("Driver")
//             .add("Trapezoid", 0.0)
//             .withWidget(BuiltInWidgets.kGraph) // specify the widget here
//             .getEntry();
//       }

//     @Override
//     public void useState(TrapezoidProfile.State setpoint) {
//         // Calculate the feedforward from the sepoint
//         double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
//         // Add the feedforward to the PID output to get the motor output
//         // articulatePIDController.setSetpoint(setpoint.position);
//         trapezoid.setDouble(setpoint.position);     
//     }
// }
