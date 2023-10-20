package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Scuff extends CommandBase{
    private DoubleSupplier xSpeed;
    private DoubleSupplier zRot;
    private DriveSubsystem swagWagon;

    public Scuff(DriveSubsystem d,DoubleSupplier a, DoubleSupplier b){
        swagWagon = d;
        addRequirements(d);
        xSpeed = a;
        zRot = b;
    }

    @Override
    public void execute(){
        swagWagon.drive(xSpeed.getAsDouble(), zRot.getAsDouble());
    }
}
