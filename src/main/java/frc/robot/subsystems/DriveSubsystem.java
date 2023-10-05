package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{

    
    private PWMSparkMax leftMotor = new PWMSparkMax(0);
    private PWMSparkMax rightMotor = new PWMSparkMax(1);
    private PWMSparkMax leftMotor2 = new PWMSparkMax(3);
    private PWMSparkMax rightMotor2 = new PWMSparkMax(4);

    private MotorControllerGroup leftGroup = new MotorControllerGroup(leftMotor, leftMotor2);
    private MotorControllerGroup rightGroup = new MotorControllerGroup(rightMotor, rightMotor2);


    private DifferentialDrive mDrivetrain = new DifferentialDrive(leftMotor, rightMotor);

    public void Drive(double xSpeed, double zRotation){
        mDrivetrain.arcadeDrive(xSpeed, zRotation);
    }


}