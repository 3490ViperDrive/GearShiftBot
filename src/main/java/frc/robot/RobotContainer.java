// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.Cruisin;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveAndShoot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;



//Class used to "describe" the robot -- subsystems, controllers, etc.
public class RobotContainer {

  private DriveSubsystem mDrive = new DriveSubsystem();
 

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

      CommandXboxController mController = new CommandXboxController(0);
      CommandJoystick mJouJoystick =  new CommandJoystick(1);

  //Constructor for initialization
  public RobotContainer() {

    mDrive.setDefaultCommand(new Cruisin(m_driverController.getRawAxis(0), m_driverController.getRawAxis(1)));


    configureBindings();
  } 

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    mController.x().onTrue(new MoveAndShoot());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public ExampleSubsystem getM_exampleSubsystem() {
    return m_exampleSubsystem;
  }

  public CommandXboxController getM_driverController() {
    return m_driverController;
  }

  public CommandXboxController getmController() {
    return mController;
  }

  public void setmController(CommandXboxController mController) {
    this.mController = mController;
  }

  public CommandJoystick getmJouJoystick() {
    return mJouJoystick;
  }

  public void setmJouJoystick(CommandJoystick mJouJoystick) {
    this.mJouJoystick = mJouJoystick;
  }
}
