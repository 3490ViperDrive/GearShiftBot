package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.arm.ArmSystem;
import frc.robot.subsystems.arm.SimArmSystem;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class ArmSubsystem extends SubsystemBase{
    private ArmSystem armSystem;

    public ArmSubsystem() {
        armSystem = (RobotBase.isReal()) ? null : new SimArmSystem(); //replace null when a real arm system is created
    }

    //closed loop control goes here somewhere (try LQR? ProfiledPID?)
    //feedforward goes here somewhere

    public void periodic() {
        armSystem.periodic();
    }

    public Command controllerMoveArmCommand(DoubleSupplier armVelocitySupplier) {
        return new RepeatCommand(new InstantCommand(() -> armSystem.setMotorOutput(armVelocitySupplier.getAsDouble()), this));
    }
}

