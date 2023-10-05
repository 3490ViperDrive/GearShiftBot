package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.lib.util.PreferencesHelper;
import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase{

    private SingleJointedArmSim armSimulator;
    private DCMotor simGearbox;
    private Mechanism2d armGUI;
    private MechanismRoot2d armGUIRoot;
    private MechanismLigament2d armGUITube;

    private ArmFeedforward feedforwardController;
    private PIDController feedbackController;
    private double lastArmPos = 0; //used for simple derivative calculation
    private double lastArmVel = 0;

    public ArmSubsystem() {
        //initialize sim components
        simGearbox = DCMotor.getNEO(1);
        armSimulator = new SingleJointedArmSim(simGearbox, 
                                                kGearRatio, 
                                                SingleJointedArmSim.estimateMOI(kLength, kMass), //replace this when better estimate is available
                                                kLength, 
                                                Units.degreesToRadians(0), 
                                                Units.degreesToRadians(140), 
                                                true);
        armSimulator.setInput(0);
        armGUI = new Mechanism2d(50, 50);
        armGUIRoot = armGUI.getRoot("arm", 25, 6);
        armGUITube = new MechanismLigament2d("arm tube",
        Units.metersToInches(kLength),
        0,
        2,
        new Color8Bit(Color.kBlue));
        armGUIRoot.append(armGUITube);
        SmartDashboard.putData("arm GUI", armGUI);
        System.out.println("MOI guestimate: " + SingleJointedArmSim.estimateMOI(kLength, kMass) + " kg/m^2"); //debug

        feedforwardController = new ArmFeedforward(Feedforward.kS, Feedforward.kG, Feedforward.kV, Feedforward.kA);
        feedbackController = new PIDController(Feedback.kP, 0, Feedback.kD /*, new TrapezoidProfile.Constraints(30, 5)*/); //number magic (deg/s? voltage?)
        feedbackController.setSetpoint(0);
        feedbackController.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        //update pid constants (use for tuning)
        /*
        feedbackController.setP(PreferencesHelper.grabDouble("Arm kP", Feedback.kP));
        feedbackController.setD(PreferencesHelper.grabDouble("Arm kD", Feedback.kD));
        */
        //manipulate setpoint
        feedbackController.setSetpoint(Units.degreesToRadians(PreferencesHelper.grabDouble("Arm setpoint", 0)));
        SmartDashboard.putNumber("Received Setpoint", Units.degreesToRadians(PreferencesHelper.grabDouble("Arm setpoint", 0)));

        armSimulator.update(0.02); //50hz
        armGUITube.setAngle(getAngleDegrees());
        //move to simulationPeriodic() if more simulated systems are added but for now it's fine here
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSimulator.getCurrentDrawAmps()));
        SmartDashboard.putNumber("Arm angle", getAngleDegrees());
    }

    public Command controllerMoveArmCommand(DoubleSupplier armVelocitySupplier) {
        return new RepeatCommand(new InstantCommand(() -> this.setMotorOutput(armVelocitySupplier.getAsDouble()), this));
    }

    public Command controllerAutoMoveArmCommand() {
        return new RepeatCommand(new InstantCommand(() -> this.setMotorOutput(calculateArmMotorOutput(MathUtil.clamp(feedbackController.calculate(getAngleRads()), -12, 12))), this));
    }
    
    public double calculateArmMotorOutput(double output) {
        double currentArmPos = getAngleRads();
        double totalOutput = MathUtil.clamp(output + feedforwardController.calculate(currentArmPos, currentArmPos - lastArmPos, (currentArmPos - lastArmPos) - lastArmVel), -12, 12);
        lastArmVel = currentArmPos - lastArmPos;
        lastArmPos = currentArmPos;
        return totalOutput;
    }

    //Does the same thing as ArmFeedforward, but the variables are more accessible here
    private double calculateFeedforwardTemp(double pos, double vel, double acc) {
        return Feedforward.kS * Math.signum(vel) + Feedforward.kG * Math.cos(pos) + Feedforward.kV * vel + Feedforward.kA * acc;
    }

    public double getAngleDegrees() {
        return Units.radiansToDegrees(getAngleRads());
    }

    public double getAngleRads() {
        if (RobotBase.isReal()) {
            return 0; //TODO make this work
        } else {
            return armSimulator.getAngleRads();
        }
        
    }
    
    public void setMotorOutput(double output) {
        SmartDashboard.putNumber("Arm power level", output);
        if (RobotBase.isReal()) {
        
        } else {
            SmartDashboard.putNumber("VIn voltage", RoboRioSim.getVInVoltage());
            armSimulator.setInput(output * RoboRioSim.getVInVoltage());
        }
    }
}

