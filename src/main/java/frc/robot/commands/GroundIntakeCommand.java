package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.ClawIntakeOption;
import frc.robot.Constants.ManipulatorSetpoint;

public class GroundIntakeCommand extends SequentialCommandGroup {
    public GroundIntakeCommand(ArmSubsystem arm, ClawSubsystem claw) {
        addRequirements(arm, claw);
        addCommands(
            arm.setSetpointCommand(ManipulatorSetpoint.kIntake),
            arm.waitUntilAtSetpointCommand(),
            claw.intakeCommand(ClawIntakeOption.kCube),
            arm.setSetpointCommand(ManipulatorSetpoint.kHybridForward),
            arm.waitUntilAtSetpointCommand()
        );
    }
}
