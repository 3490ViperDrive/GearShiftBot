package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

public class Cruisin extends CommandBase{

    DriveSubsystem mDriveSubsystem = new DriveSubsystem();

        private double curSpeed;
        private double curRotation;

        public Cruisin(double xSpeed, double zRotation){

            this.curSpeed = xSpeed;
            this.curRotation = zRotation;
        }
    
    @Override
    public void execute(){
        mDriveSubsystem.tankDrive(curSpeed, curRotation);
    }
    
}
