package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Shooter;

public class MoveAndShoot extends CommandBase{
    public void execute(){
        DriveSubsystem.tankDrive(.7, .7);
        Shooter.fireShot();
    }
}
