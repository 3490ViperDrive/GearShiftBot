package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlexSystem;

public class Vroom extends CommandBase{

    public void execute(double one, double two){
        DriveSubsystem.tankDrive(one, two);
        FlexSystem.strobeTheLights();
    }
}
