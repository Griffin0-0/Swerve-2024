package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

public class WaitForTimeCmd extends Command {

    private double startTime;
    private double endTime; 

    public WaitForTimeCmd(double seconds) {
        this.startTime = Timer.getFPGATimestamp();
        this.endTime = startTime + seconds;
    }

    public boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTime;
    }
}
