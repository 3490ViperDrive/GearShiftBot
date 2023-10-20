package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.ClawConstants.*;

import frc.lib.util.PreferencesHelper;
import frc.robot.Constants.ManipulatorSetpoint;

public class ClawSubsystem extends SubsystemBase {
    private DoubleSolenoid clawPneumatic;
    private CANSparkMax leftSparkMax;
    private CANSparkMax rightSparkMax;
    private ClawIntakeOption intakeState;

    private MedianFilter leftSparkMaxCurrentFilter;
    private MedianFilter rightSparkMaxCurrentFilter;
    private Timer timer;

    double leftCurrentMedian;
    double rightCurrentMedian;

    public enum ClawIntakeOption {
        kNone,
        kCube,
        kCone
    }
    
    public ClawSubsystem() {
        //real components
        leftSparkMax = new CANSparkMax(kClawLeftMotorControllerID, MotorType.kBrushless);
        rightSparkMax = new CANSparkMax(kClawRightMotorControllerID, MotorType.kBrushless);
        clawPneumatic = new DoubleSolenoid(PneumaticsModuleType.REVPH, kClawSolenoidForwardChannelID, kClawSolenoidReverseChannelID);
        leftSparkMax.setInverted(true);
        rightSparkMax.setInverted(true);
        leftSparkMax.setSmartCurrentLimit(kClawMotorCurrentLimit); //no current limit can easily burn out a 550
        rightSparkMax.setSmartCurrentLimit(kClawMotorCurrentLimit);
        leftSparkMax.setIdleMode(IdleMode.kBrake); //is this optimal?
        rightSparkMax.setIdleMode(IdleMode.kBrake);
        rightSparkMax.follow(leftSparkMax, true); //rollers should spin opposite of one another, change invert if necessary

        intakeState = ClawIntakeOption.kNone;

        leftSparkMaxCurrentFilter = new MedianFilter(3); //increase the size of these filters if any intake false flags happen during intake
        rightSparkMaxCurrentFilter = new MedianFilter(3);
        //debouncer = new Debouncer(1, DebounceType.kRising);
        timer = new Timer();

        clawPneumatic.set(Value.kOff);
    }

    @Override
    public void periodic() {
        leftCurrentMedian = leftSparkMaxCurrentFilter.calculate(leftSparkMax.getOutputCurrent());
        rightCurrentMedian = rightSparkMaxCurrentFilter.calculate(rightSparkMax.getOutputCurrent());

        SmartDashboard.putNumber("Claw Avg Motor Controller Current", (leftSparkMax.getOutputCurrent() + rightSparkMax.getOutputCurrent()) / 2);
        SmartDashboard.putNumber("Claw Avg Motor Controller Current (Filtered)", (leftCurrentMedian + rightCurrentMedian) / 2);
    }

    public void setClawState(ClawIntakeOption holding) {
        if (!DriverStation.isTeleop()) {
            intakeState = holding;
        }
    }

    public ClawIntakeOption getClawState() {
        return intakeState;
    }
    
    /**Returns a command that runs the intake until it acquires a game piece or the command is cancelled.
     * It will not do anything if the intake already contains a game piece.
     * @param gamepiece whether to open the claw wide enough for a cube or cone
     * @return the command
     */
    public Command intakeCommand(ClawIntakeOption gamePiece) {
        return new FunctionalCommand(
            () -> {
                timer.reset();
                if (intakeState == null) {
                    System.out.println("intakeState is null, bruh!!!");
                    return;
                }
                if (intakeState != ClawIntakeOption.kNone) return; //claw already has a game piece, why acquire another one?
                /*
                if (gamePiece == ClawIntakeOption.kCube) {
                    clawPneumatic.set(Value.kForward);
                } else if (gamePiece == ClawIntakeOption.kCone) {
                    clawPneumatic.set(Value.kReverse);
                }*/
                leftSparkMax.set(ManipulatorSetpoint.kIntake.intakeSpeed);
                
                //System.out.println("Claws goin");
            },
            () -> {
                
            },
            (interrupted) -> {
                leftSparkMax.set(0);
            },
            () -> {
                //fix this
                if ((leftCurrentMedian >= kClawStallCurrentThreshold || rightCurrentMedian >= kClawStallCurrentThreshold) && timer.get() > kClawStallCurrentReadDebounce) { //game piece hits the back wall of the claw
                    intakeState = gamePiece;
                    //System.out.println("Intaked gamepiece");
                    return true;
                } else if (intakeState != ClawIntakeOption.kNone) {
                    //System.out.println("Intake attempted, but claw is already full");
                    return true;
                } else {
                    return false;
                }
            }
        );
    }

    public Command shootCommand(ClawIntakeOption gamePiece, ManipulatorSetpoint position) {
        return new FunctionalCommand(
            () -> {
                //if (intakeState == ClawIntakeOption.kNone || position == ManipulatorSetpoint.kIntake) return;
                leftSparkMax.set(position.intakeSpeed);
            },
            () -> {},
            (interrupted) -> {
                leftSparkMax.set(0);
                intakeState = ClawIntakeOption.kNone;
            },
            () -> {return false;}
        ).withTimeout(1); //adjust this timeout if needed
    }
}
