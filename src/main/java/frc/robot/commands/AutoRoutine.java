package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoRoutine extends CommandBase{
    private  double m_startTime, m_time;
    private double m_autoTime = 15;
    
    @Override
    public void initialize(){
    //init
    m_startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
        m_time = Timer.getFPGATimestamp() - m_startTime;
    // if time is less than how long you want to drive in auto, set voltage at 50%
    // if time is greater than how long you want to drive, set voltage to 0%
    if (m_time < m_autoTime) {
      //robot drives forward at 50 percengt
    } else {
      //stop the robot
    }
    }

    @Override
    public void end(boolean interrupted){
    }
}
