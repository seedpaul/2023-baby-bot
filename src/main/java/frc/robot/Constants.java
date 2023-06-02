// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class LEDConstants{
    public static final int LED_PWM_PORT = 0;
    public static final int LED_LENGTH = 46;

    public static final Color purple = new Color(255, 1, 255);
    public static final Color yellow = new Color(255, 128, 1);
    public static final Color orange = new Color(255, 55, 5);
    public static final Color pink = new Color(255, 50, 30);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kAssistControllerPort = 1;
    public static final int A_BUTTON = 1;
    public static final int B_BUTTON = 2;
    public static final int X_BUTTON = 3;
    public static final int Y_BUTTON = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int TWO_SQUARES_BUTTON = 7;
    public static final int ADIDAS_BUTTON = 8;
    public static final int LEFT_STICK_PRESS = 9;
    public static final int RIGHT_STICK_PRESS = 10;
  }

  public static class DriveBaseConstants {
    public static final int leftBackTalonFX_ID = 41;
    public static final int leftFrontTalonFX_ID = 42;
    public static final int rightBackTalonFX_ID = 21;
    public static final int rightFrontTalonFX_ID = 24;

    public static final double rotationGoverner = 1.0;
    public static final double speedGoverner = 1.0;

    public static final double wheelcircumference = 0.49149; // meters
    public static final double cpr = 2048; // count per rotation for talonFX integrated encoder
    private static final double gearRatio = 8.41; // to 1
    private static final double cpWheelr = cpr * gearRatio; // (17223.68) count per rotation for talonFX integrated
                                                            // encoder for each full wheel rotation
    public static final double distancePerPulse = wheelcircumference / cpWheelr;

    public static final double balanceSpeedMultipier = 0.0625;
    public static final int balanceRollThreshold = 3;
    public static final double balanceSpeed = .65;
  }

  public static class SnatchAndSpitConstants {
    public static final int upperSparkMaxID = 33;
    public static final int lowerSparkMaxID = 9;

    public static final class UpperPIDConstants {
      public static final double kP = 1.0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kFF = 0.0;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
      public static final double maxRPM = 5700;
      public static final double maxVel = 2000;
      public static final double minVel = 0;
      public static final double maxAcc = 1500;
      public static final int smartMotionSlot = 0;
      public static final double allowedError = 0;
    }

    public static final class LowerPIDConstants {
      public static final double kP = 1.0;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kIz = 0;
      public static final double kFF = 0.0;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
      public static final double maxRPM = 5700;
      public static final double maxVel = 2000;
      public static final double minVel = 0;
      public static final double maxAcc = 1500;
      public static final int smartMotionSlot = 0;
      public static final double allowedError = 0;
    }

    public static final class VelocityConstants {
      public static final class Upper{
        public static final double intake = 1000;
        public static final double shootHigh = 5200;
        public static final double shootMiddle = 3000;
        public static final double drop = 500;
        public static final double [] validVelocity = {intake,shootHigh,shootMiddle,drop};
      }
      public static final class Lower{
        public static final double intake = 1000;
        public static final double shootHigh = 5200;
        public static final double shootMiddle = 3000;
        public static final double drop = 500;
        public static final double [] validVelocity = {intake,shootHigh,shootMiddle,drop};
      }
    }
  }

  public static class ElbowConstants {

    public static final int elbowSparkMaxRightID = 11;
    public static final int elbowSparkMaxLeftID = 55;
    public static final int elbowCANCoderID = 13;

    public static final class ElbowPIDConstants {
      public static final double kP = 0.0075;
      public static final double kI = 0;
      public static final double kD = 0;
      public static final double kMaxOutput = 1;
      public static final double kMinOutput = -1;
      public static final int smartMotionSlot = 0;
      public static final double allowedError = 0;
    }

    public static final class ElbowPositionConstants {

      private static final int zeroOffset = 30;
      public static final double magnetOffsetDegrees = -66.5 + zeroOffset;
      public static final double starting = -10 + zeroOffset;
      //public static final double shooting = 36 + zeroOffset;
      public static final double drop = 71 + zeroOffset;
      public static final double intake = 107 + zeroOffset;
      //public static final double [] positions = {starting,shooting,drop,intake};
      public static final double [] positions = {starting,drop,intake};
    } 
  }

  public static final class AutoDriveConstants {

    public static final double kTrackwidthMeters = 1.4926;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double ksVolts = 0.73483;
    public static final double kvVoltSecondsPerMeter = 1.8058;
    public static final double kaVoltSecondsSquaredPerMeter = 0.30098;

    public static final double kPDriveVel = 0.00030372; //0.3872;

    public static final double kMaxSpeedMetersPerSecond = 3;// default value 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;// default value 3;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
}