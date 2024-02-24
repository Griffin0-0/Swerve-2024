package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;

public class IntakeSubsystem extends SubsystemBase {

    private final CANSparkMax intakeMotor;
    private final CANSparkMax articulateMotor;
    private final SparkPIDController articulatePID;
    private final RelativeEncoder articulateEncoder;
    private final GenericEntry devsb_encoder;
    private final LEDSubsystem ledSubsystem;
    
    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    private final Color noteColor = new Color(0.51, 0.36, 0.13);
    private final double tolerance = 0.15;

    public boolean intakeOut = false;
    public boolean switchTemp = false;
    public double currentGoal = 0.0;
    public boolean noteConfirmed = false;


    public IntakeSubsystem(LEDSubsystem ledSubsystem) {
        intakeMotor = new CANSparkMax(IntakeConstants.kIntakeMotorId, MotorType.kBrushless);

        articulateMotor = new CANSparkMax(IntakeConstants.kIntakeArticulateMotorId, MotorType.kBrushless);
        articulatePID = articulateMotor.getPIDController();
        articulateEncoder = articulateMotor.getEncoder();
        articulateEncoder.setPosition(0);

        articulatePID.setOutputRange(-IntakeConstants.kIntakeArticulateSpeed, IntakeConstants.kIntakeArticulateSpeed);
        articulatePID.setP(0.6);
        articulatePID.setI(0);
        articulatePID.setD(0);

        articulateEncoder.setPosition(0);

        devsb_encoder = Shuffleboard.getTab("Driver")
            .add("Encoder Pos", 0.0)
            .withPosition(0, 1)
            .withSize(3, 1)
            .getEntry();

        this.ledSubsystem = ledSubsystem;
    }

    public void intakeDown() {
        // currentGoal = IntakeConstants.kIntakeDesiredPos_out;
        articulatePID.setReference(IntakeConstants.kIntakeDesiredPos_out, ControlType.kPosition);   
        intakeOut = true;
    }
    public void intakeUp() {
        // currentGoal = IntakeConstants.kIntakeDesiredPos_store;
        articulatePID.setReference(IntakeConstants.kIntakeDesiredPos_store, ControlType.kPosition);
        intakeOut = false;
    }
    public void intakeAmp() {
        // currentGoal = IntakeConstants.kIntakeDesiredPos_amp;
        articulatePID.setReference(IntakeConstants.kIntakeDesiredPos_amp, ControlType.kPosition);
    }

    public void runIntake(double speed) {
        intakeMotor.set(speed);
    }

    public void toggleIntake() {
        if (intakeOut) {
            intakeUp();
        } else if (!intakeOut) {
            intakeDown();
        }
    }

    public boolean atPoint(double point) {
        return (Math.abs(getPosition() - point) < 0.5);
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
        runIntake(IntakeConstants.kIntakeMotorSpeed_ground);
    }

    public void spinOut() {
        runIntake(IntakeConstants.kIntakeMotorSpeed_out);
    }

    public void spinAmp() {
        runIntake(IntakeConstants.kIntakeMotorSpeed_amp);
    }

    public double getPosition() {
        return articulateEncoder.getPosition();
    }

    @Override
    public void periodic() {
        devsb_encoder.setDouble(getPosition());

        if (isDown()) {
            spinIn();
            switchTemp = true;
            ledSubsystem.setIntake();
        } else if (switchTemp) {
            stopIntake();
            switchTemp = false;
            ledSubsystem.setDefault();
        }

        Color detectedColor = m_colorSensor.getColor();   

        double redDifference = noteColor.red - detectedColor.red;
        double greenDifference = noteColor.green - detectedColor.green;
        double blueDifference = noteColor.blue - detectedColor.blue;

        noteConfirmed = (Math.abs(redDifference) < tolerance) && (Math.abs(greenDifference) < tolerance) && (Math.abs(blueDifference) < tolerance);

        SmartDashboard.putNumber("color red", detectedColor.red);
        SmartDashboard.putBoolean("note comfirmed", noteConfirmed);
        if (noteConfirmed && isDown()) {
            intakeUp();
            intakeOut = false;
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
