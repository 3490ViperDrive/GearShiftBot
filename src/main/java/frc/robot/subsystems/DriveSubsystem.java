package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase{

    //Drivetrain will need *motor controllers* at the very least. 
    //A gyro may also be advisable for any sort of software-assisted navigation.

    private PWMSparkMax leftMotor1 = new PWMSparkMax(0);
    private PWMSparkMax leftMotor2 = new PWMSparkMax(1);
    private PWMSparkMax rightMotor1 = new PWMSparkMax(2);
    private PWMSparkMax rightMotor2 = new PWMSparkMax(3);

    private  MotorControllerGroup leftMotors;
    private MotorControllerGroup rightMotors;


    public void tankDrive(double leftSpeed, double rightSpeed){


        leftMotors = new MotorControllerGroup(leftMotor1, leftMotor2, null);
        rightMotors = new MotorControllerGroup(rightMotor1, rightMotor2, null);

    }

    


    



    



    @Override
    public void periodic(){

    }
}
