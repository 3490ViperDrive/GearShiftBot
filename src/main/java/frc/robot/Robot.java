// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
   private PWMSparkMax m_rightMaster, m_leftMaster, m_leftSlave, m_rightSlave;
  private Joystick m_leftStick, m_rightStick;
  private double m_startTime, m_time;
  private double m_autoTime = 5;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_leftStick = new Joystick(0);
    m_rightStick = new Joystick(1);
                                                    // most of that needs to be deleted or changed for DriveSubsystem.java [at least I think so idk :shrug:]
    // Invert the left and right sides
    m_leftMaster.setInverted(false);
    m_rightMaster.setInverted(true);

    m_leftSlave.setInverted(false);
    m_rightSlave.setInverted(true);

    // Make the slave motors follow the master

    // m_leftSlave.follow(m_leftMaster); 
    // m_rightSlave.follow(m_rightMaster);

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_startTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_time = Timer.getFPGATimestamp() - m_startTime;
    //if time is less than how long you want to drive in auto, set voltage at 50%
    //if time is greater than how long you want to drive, set voltage to 0%

    // The code below needs fixing with what I have coded in DriveSubsystem.java


    
    // if (m_time < m_autoTime) {
    //   m_rightMaster.set(ControlMode.PercentOutput, 0.5);
    //   m_leftMaster.set(ControlMode.PercentOutput, 0.5);
    // } else {
    //   // Stops robot
    //   m_leftMaster.set(ControlMode.PercentOutput, 0);
    //   m_rightMaster.set(ControlMode.PercentOutput, 0);
    // }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //tankDrive();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
