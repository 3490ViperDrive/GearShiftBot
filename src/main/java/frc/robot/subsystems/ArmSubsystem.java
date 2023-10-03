package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmSystem;
import frc.robot.subsystems.arm.SimArmSystem;

public class ArmSubsystem extends SubsystemBase{
    private ArmSystem armSystem;

    public ArmSubsystem() {
        armSystem = (RobotBase.isReal()) ? null : new SimArmSystem(); //replace null when a real arm system is created
    }

    //closed loop control goes here somewhere (try LQR? ProfiledPID?)
    //feedforward goes here somewhere
}
