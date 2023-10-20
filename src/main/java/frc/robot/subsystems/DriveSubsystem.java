package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import static frc.robot.Constants.DrivetrainConstants.*;

public class DriveSubsystem extends SubsystemBase {

   // ¯\_(ツ)_/¯
    private CANSparkMax leftMotor;
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor2;
    private CANSparkMax rightMotor2;

    private Solenoid leftGearboxSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 0);
    private Solenoid rightGearboxSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 1);

    // Creates a group for the left motors and right motors, basically adds the ability to steer instead of doing some complex mathmatics or something
    //private MotorControllerGroup leftGroup;
    //private MotorControllerGroup rightGroup;

    //private DifferentialDrive drivetrain;

    private GenericHID xboc;

    public DriveSubsystem() {
        leftMotor = new CANSparkMax(kFrontLeftMotorController, MotorType.kBrushless);
        rightMotor = new CANSparkMax(kFrontRightMotorController, MotorType.kBrushless);
        leftMotor2 = new CANSparkMax(kBackLeftMotorController, MotorType.kBrushless);
        rightMotor2 = new CANSparkMax(kBackRightMotorController, MotorType.kBrushless);

        configDriveMotor(leftMotor);
        configDriveMotor(rightMotor);
        configDriveMotor(leftMotor2);
        configDriveMotor(rightMotor2);

        leftMotor2.follow(leftMotor);
        rightMotor2.follow(rightMotor);

        xboc = new GenericHID(0);

        //leftGroup = new MotorControllerGroup(leftMotor, leftMotor2);
        //rightGroup = new MotorControllerGroup(rightMotor, rightMotor2);
        //rightGroup.setInverted(true);
        //drivetrain = new DifferentialDrive(leftGroup, rightGroup);

        leftGearboxSolenoid.set(true);
        rightGearboxSolenoid.set(true);
    }

    private void configDriveMotor(CANSparkMax motor) {
        motor.setIdleMode(IdleMode.kBrake); //is this ideal?
        motor.setSmartCurrentLimit(40);
    }
    
    // All the code below has the purpose of just being there to set the drive train into action or set the properties of it [Named mDrivetrain]. I got nothing else ¯\_(ツ)_/¯

    @Override
    public void periodic(){
    }

    public void drive(double xSpeed, double zRotation){
        //observed: up on left stick turns right side backwards, down on left stick turns right side forwards
        leftMotor.set(xSpeed);
        rightMotor.set(zRotation);
        SmartDashboard.putNumber("left motor power", xSpeed);
    }

    public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier zRotation) {
        return new RepeatCommand(new InstantCommand(() -> {drive(xSpeed.getAsDouble(), zRotation.getAsDouble());}, this));
    }
}
