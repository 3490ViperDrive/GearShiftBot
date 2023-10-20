// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DrivetrainConstants{
    public static final int kFrontLeftMotorController = 1; //this should be accurate to the robot
    public static final int kFrontRightMotorController = 3;
    public static final int kBackLeftMotorController = 2;
    public static final int kBackRightMotorController = 4;
  }

  public static final class ArmConstants {
    public static final int kArmMotorControllerID = 5;
    public static final int kArmEncoderChannel = 0; //DIO port
    public static final double kArmEncoderOffset = 30; //TODO find this when encoder is mounted

    public static final class Feedforward { //Data from ReCalc; SysID the real consts as soon as the robot is accessible
      public static final double kG = 0.034;
      public static final double kV = 1.91;
      public static final double kA = 0.01;
      public static final double kS = 0.01;
    }

    public static final class Feedback {
      public static final double kP = 0.1;
      public static final double kD = 0;
    }

    //these are to be used for simulation
    public static final double kGearRatio = 98; //49:1 versaplanetary + 2:1 chain; guessed
    public static final double kLength = Units.inchesToMeters(16); //guessed, replace when better estimate is available
    public static final double kMass = Units.lbsToKilograms(10); //guessed
    public static final double kRotationalInertia = 0; //aka Moment of Inertia; this will have to be checked in CAD later

  }

  public static final class ClawConstants {
    public static final int kClawMotorCurrentLimit = 20;
    public static final int kClawSolenoidForwardChannelID = 14;
    public static final int kClawSolenoidReverseChannelID = 15;
    public static final int kClawLeftMotorControllerID = 6;
    public static final int kClawRightMotorControllerID = 7;

    public static final double kClawStallCurrentThreshold = 5; //TODO find on robot
    public static final double kClawStallCurrentReadDebounce = 1; //second(s)
  }

  public enum ManipulatorSetpoint {
    kIntake(-5, -0.45),    //these are all guesses, update to true values later
    kHybridForward(0, 0.5), //also the stow position
    kL2Forward(40, 0.7),
    kL3Forward(50, 1),
    kHybridReverse(135, 0.2),
    kL2Reverse(125, 0.7),
    kL3Reverse(120, 1);

    public final double angle;
    public final double intakeSpeed; //[-1, 1]

    ManipulatorSetpoint(double angle, double intakeSpeed) {
        this.angle = angle;
        this.intakeSpeed = intakeSpeed;
    }

    public boolean isAtPosition(double angle) {
      return angle < this.angle + 2 && angle > this.angle - 2;
    }
  }
}

