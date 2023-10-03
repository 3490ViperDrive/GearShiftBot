package frc.robot.subsystems.arm;

/**
 * The inputs and outputs of the robot's arm. Encompasses sensor input, motor input and output, filtering, etc.
 * May be real or simulated.
 */
public abstract class ArmSystem {
    
    public ArmSystem () {

    }

    /**Returns the angle of the arm in degrees*/
    public abstract double getAngle();

    /**Sets the output of the motor connected to the arm in volts*/
    public abstract void setMotorOutput(double output);

    /**Put this in the ArmSubsystem periodic*/
    public abstract void periodic();
}
