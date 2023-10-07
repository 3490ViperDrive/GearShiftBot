package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class DriveSubsystem extends SubsystemBase{

   // Basically this creates the motor's below and adds a certain channel id to each... I don't know what else there is to it ¯\_(ツ)_/¯
    private PWMSparkMax leftMotor = new PWMSparkMax(0);
    private PWMSparkMax rightMotor = new PWMSparkMax(1);
    private PWMSparkMax leftMotor2 = new PWMSparkMax(3);
    private PWMSparkMax rightMotor2 = new PWMSparkMax(4);

    // Creates a group for the left motors and right motors, basically adds the ability to steer instead of doing some complex mathmatics or something
    private MotorControllerGroup leftGroup = new MotorControllerGroup(leftMotor, leftMotor2);
    private MotorControllerGroup rightGroup = new MotorControllerGroup(rightMotor, rightMotor2);
    private XboxController DrivingController = new XboxController(1);




    @Override
    public void robotInit(){
        rightGroup.setInverted(true);
    }
     
    
    @Override
    public void teleopPeriodic(){
        Drive.arcadeDrive(-DrivingController.getLeftY(), -DrivingController.getRightX());
    }
    
    
    // All the code below has the purpose of just being there to set the drive train into action or set the properties of it [Named mDrivetrain]. I got nothing else ¯\_(ツ)_/¯
    private DifferentialDrive mDrivetrain = new DifferentialDrive(leftMotor, rightMotor);

    public void Drive(double xSpeed, double zRotation){
         mDrivetrain.arcadeDrive(xSpeed, zRotation);
    }


}

/*
 * TODO - 
 * 
 * 
 *        Create a way for the xbox controller to read off the inputs to the code so that the robot moves forward.
 *         
 *              Clean up any mess I make along the way.
 * 
 *                      Do not complicate and change my code to make it really stupid hard to read or understand and make notes.
 * 
 *                              Do not do anything stupid with the code I have and try using Adamya's code as a base for what I can do and change it for xbox controllers.
 * 
 * 
 * 
 *             Don't do anything dumb when writing out the code.
 * 
 * 
 * 
 *                          READ ME:
 *                                      If you're looking through my code and this message is here, then nothing I do is permanent and is subject to change.
 */