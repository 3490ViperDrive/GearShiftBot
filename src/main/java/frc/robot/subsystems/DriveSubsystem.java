package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;



public class DriveSubsystem extends SubsystemBase{

   // ¯\_(ツ)_/¯
    private PWMSparkMax leftMotor = new PWMSparkMax(kFrontLeftMotorController);
    private PWMSparkMax rightMotor = new PWMSparkMax(kFrontRightMotorController);
    private PWMSparkMax leftMotor2 = new PWMSparkMax(kBackLeftMotorController);
    private PWMSparkMax rightMotor2 = new PWMSparkMax(kBackRightMotorController);

    // Creates a group for the left motors and right motors, basically adds the ability to steer instead of doing some complex mathmatics or something
    private MotorControllerGroup leftGroup = new MotorControllerGroup(leftMotor, leftMotor2);
    private MotorControllerGroup rightGroup = new MotorControllerGroup(rightMotor, rightMotor2);

    private DifferentialDrive mDrivetrain = new DifferentialDrive(leftMotor, rightMotor);






    public void robotInit(){
        rightGroup.setInverted(true);
    }
     
    
    public void teleopPeriodic(){
    }
    
    
    // All the code below has the purpose of just being there to set the drive train into action or set the properties of it [Named mDrivetrain]. I got nothing else ¯\_(ツ)_/¯
    @Override
    public void periodic(){
        
    }

    public void Drive(double xSpeed, double zRotation){
         mDrivetrain.arcadeDrive(xSpeed, zRotation);
    }


}
