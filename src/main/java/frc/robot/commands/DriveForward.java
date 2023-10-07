package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class DriveForward extends CommandBase{
    // Something that makes this run and check's what the controller is doing to change inputs and send values
    @Override
    public void execute(){
        
        /* 
        Concept code below showing how this might work and how it can get the Y value of the joystick to change the left and right group [right now seperatly].

        Most code below probably will not work and needs fixing with proper syntax and xbox control's [or smth else idk at]
        */ 
        // leftGroup.set(ControlMode.PercentOutput, -m_leftStick.getY());
        // rightGroup.set(ControlMode.PercentOutput, -m_rightStick.getY());
    }
}


/* proof of concept code vvv

 * public void tankDrive() {
      // directly set the joystick y axes to the motors
      m_leftMaster.set(ControlMode.PercentOutput, -m_leftStick.getY());
      m_rightMaster.set(ControlMode.PercentOutput, -m_rightStick.getY());
    }
 */