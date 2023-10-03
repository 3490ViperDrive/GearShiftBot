package frc.robot.subsystems.arm;

import static frc.robot.Constants.ArmConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SimArmSystem extends ArmSystem {

    private SingleJointedArmSim armSimulator;
    private DCMotor gearbox = DCMotor.getNEO(1);

    public SimArmSystem() {
        armSimulator = new SingleJointedArmSim(gearbox, 
                                                kGearRatio, 
                                                SingleJointedArmSim.estimateMOI(kLength, kMass), //replace this when better estimate is available
                                                kLength, 
                                                Units.degreesToRadians(0), 
                                                Units.degreesToRadians(140), 
                                                true);
        armSimulator.setInput(0);
        System.out.println("MOI guestimate: " + SingleJointedArmSim.estimateMOI(kLength, kMass) + " kg/m^2");
    }

    public void periodic() {
        armSimulator.update(0.02);
        //move to simulationPeriodic() if more simulated systems are added but for now it's fine here
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(armSimulator.getCurrentDrawAmps()));
    }

    public double getAngle() {
        return Units.degreesToRadians(armSimulator.getAngleRads());
    }
    
    public void setMotorOutput(double output) {
        armSimulator.setInput(output);
    }
}

