package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.PreferencesHelper;

import static frc.robot.Constants.ArmConstants.*;
import frc.robot.Constants.ManipulatorSetpoint;

public class ArmSubsystem extends SubsystemBase{

    private SingleJointedArmSim armSimulator;
    private DCMotor simGearbox;
    private Mechanism2d armGUI;
    private MechanismRoot2d armGUIRoot;
    private MechanismLigament2d armGUITube;

    private ArmFeedforward feedforwardController;
    private PIDController feedbackController;
    private double lastArmPos = 0; //used for simple derivative calculation in simulation
    private double lastArmVel = 0;

    private CANSparkMax armSparkMax;
    private DutyCycleEncoder armAbsEncoder;

    

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

        //initialize real components
        armSparkMax = new CANSparkMax(kArmMotorControllerID, MotorType.kBrushless);
        armAbsEncoder = new DutyCycleEncoder(kArmEncoderChannel);
        armSparkMax.setIdleMode(IdleMode.kBrake); //is this optimal?
        armSparkMax.setInverted(false); // TODO find this on robot

        feedforwardController = new ArmFeedforward(Feedforward.kS, Feedforward.kG, Feedforward.kV, Feedforward.kA);
        feedbackController = new PIDController(Feedback.kP, 0, Feedback.kD /*, new TrapezoidProfile.Constraints(30, 5)*/); //number magic (deg/s? voltage?)
        feedbackController.setSetpoint(0);
        feedbackController.setTolerance(Units.degreesToRadians(1));
    }

    @Override
    public void periodic() {
        //assume pid is always enabled
        setMotorOutput(calculateArmMotorOutput(MathUtil.clamp(feedbackController.calculate(getAngleRads()), -12, 12)));
        updateDashboardData();
    }

    @Override
    public void simulationPeriodic() {
        //manipulate setpoint
        if (debugModeEnabled()) {
            //update pid constants (use for tuning)
            feedbackController.setP(PreferencesHelper.grabDouble("Arm kP", Feedback.kP));
            feedbackController.setD(PreferencesHelper.grabDouble("Arm kD", Feedback.kD));
            feedbackController.setSetpoint(Units.degreesToRadians(PreferencesHelper.grabDouble("Arm setpoint (Debug Mode only)", 0)));
        }
        //setMotorOutput(calculateArmMotorOutput(MathUtil.clamp(feedbackController.calculate(getAngleRads()), -12, 12)));

        armSimulator.update(0.02); //50hz
        armGUITube.setAngle(getAngleDegrees());
        //move to robot.java simulationPeriodic() if more simulated systems are added but for now it's fine here
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSimulator.getCurrentDrawAmps()));
        //updateDashboardData();
    }

    private void updateDashboardData() {
        SmartDashboard.putNumber("Arm angle", getAngleDegrees());
        SmartDashboard.putNumber("Setpoint", feedbackController.getSetpoint());
        SmartDashboard.putBoolean("At Setpoint?", feedbackController.atSetpoint());
    }

    //this should probably be in RobotContainer.java
    public static boolean debugModeEnabled() {
        //Debug Mode should NEVER be enabled on the field
        return (!DriverStation.isFMSAttached()) && PreferencesHelper.grabBoolean("Debug Mode enabled?", false);
    }

    /**Returns a Command that sets the setpoint of the arm's feedback controller.
     * @param setpoint the angle in degrees that the arm should go to
     * @return the command
     */
    public Command setSetpointCommand(double setpoint) {
        return new InstantCommand(() -> {this.feedbackController.setSetpoint(Units.degreesToRadians(setpoint));});
    }

    /**Returns a Command that sets the setpoint of the arm's feedback controller.
     * @param setpoint the ManipulatorSetpoint that the arm should go to
     * @return the command
     */
    public Command setSetpointCommand(ManipulatorSetpoint setpoint) {
        return setSetpointCommand(setpoint.angle);
    }

    /**Returns a Command that ends once the arm reaches its setpoint
     * @return the command
     */
    public Command waitUntilAtSetpointCommand() {
        return new SequentialCommandGroup(new WaitCommand(0.05), //debounce to prevent false positives
                                            new WaitUntilCommand(feedbackController::atSetpoint));
    }

    /*
    public Command controllerMoveArmCommand(DoubleSupplier armVelocitySupplier) {
        return new RepeatCommand(new InstantCommand(() -> this.setMotorOutput(armVelocitySupplier.getAsDouble()), this));
    }

    public Command controllerAutoMoveArmCommand() {
        return new RepeatCommand(new InstantCommand(() -> this.setMotorOutput(calculateArmMotorOutput(MathUtil.clamp(feedbackController.calculate(getAngleRads()), -12, 12))), this));
    }
    */

    public double calculateArmMotorOutput(double output) {
        double currentArmPos = getAngleRads();
        //guestimating velocity like this is awful, fix soon
        double totalOutput = MathUtil.clamp(output + feedforwardController.calculate(currentArmPos, currentArmPos - lastArmPos/*, (currentArmPos - lastArmPos) - lastArmVel*/), -12, 12);
        lastArmVel = currentArmPos - lastArmPos;
        lastArmPos = currentArmPos;
        return totalOutput;
    }

    public double getAngleDegrees() {
        return Units.radiansToDegrees(getAngleRads());
    }

    public double getAngleRads() {
        if (RobotBase.isReal()) {
            return ((armAbsEncoder.getAbsolutePosition() * 360) % 360) - kArmEncoderOffset; //TODO check if this works on the robot
        } else {
            return armSimulator.getAngleRads();
        }
        
    }

    //[-12, 12] voltage
    public void setMotorOutput(double output) {
        SmartDashboard.putNumber("Arm power level", output);
        if (RobotBase.isReal()) {
            //TODO figure out a good soft limits solution
            armSparkMax.setVoltage(output);
        } else if (!DriverStation.isDisabled()) {
            SmartDashboard.putNumber("VIn voltage", RoboRioSim.getVInVoltage());
            armSimulator.setInput(output * RoboRioSim.getVInVoltage() / 12);
        }
    }
}

