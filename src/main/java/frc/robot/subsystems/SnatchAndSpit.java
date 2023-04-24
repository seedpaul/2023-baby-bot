// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SnatchAndSpit extends SubsystemBase {

  private static SnatchAndSpit sas = null;
  private double TargetVelocityLower = .0;
  private double TargetVelocityUpper = .0;

  private final RelativeEncoder encoderLower;
  private final RelativeEncoder encoderUpper;

  private SparkMaxPIDController pidControllerUpper;
  private SparkMaxPIDController pidControllerLower;

  private final CANSparkMax lowerSparkMax = new CANSparkMax(Constants.SnatchAndSpitConstants.lowerSparkMaxID,
      MotorType.kBrushed);
  private final CANSparkMax upperSparkMax = new CANSparkMax(Constants.SnatchAndSpitConstants.upperSparkMaxID,
      MotorType.kBrushed);

  private int currentRPMIndex = 0;

  public static SnatchAndSpit getInstance(){
    if (sas == null){
      sas = new SnatchAndSpit();
    }
    return sas;

  }

  /** Creates a new SnatchAndSpit. */
  private SnatchAndSpit() {

    lowerSparkMax.restoreFactoryDefaults();
    upperSparkMax.restoreFactoryDefaults();

    lowerSparkMax.setIdleMode(IdleMode.kCoast);
    upperSparkMax.setIdleMode(IdleMode.kCoast);

    lowerSparkMax.enableVoltageCompensation(12);
    upperSparkMax.enableVoltageCompensation(12);

    encoderLower = lowerSparkMax.getEncoder();
    encoderUpper = upperSparkMax.getEncoder();

    pidControllerLower = lowerSparkMax.getPIDController();
    pidControllerUpper = upperSparkMax.getPIDController();

    pidControllerLower.setFeedbackDevice(encoderLower);
    pidControllerUpper.setFeedbackDevice(encoderUpper);

    pidControllerLower.setP(Constants.SnatchAndSpitConstants.LowerPIDConstants.kP);
    pidControllerLower.setI(Constants.SnatchAndSpitConstants.LowerPIDConstants.kI);
    pidControllerLower.setD(Constants.SnatchAndSpitConstants.LowerPIDConstants.kD);
    pidControllerLower.setIZone(Constants.SnatchAndSpitConstants.LowerPIDConstants.kIz);
    pidControllerLower.setFF(Constants.SnatchAndSpitConstants.LowerPIDConstants.kFF);
    pidControllerLower.setOutputRange(Constants.SnatchAndSpitConstants.LowerPIDConstants.kMinOutput,
        Constants.SnatchAndSpitConstants.LowerPIDConstants.kMaxOutput);
    pidControllerLower.setSmartMotionMaxVelocity(Constants.SnatchAndSpitConstants.LowerPIDConstants.maxVel,
        Constants.SnatchAndSpitConstants.LowerPIDConstants.smartMotionSlot);
    pidControllerLower.setSmartMotionMinOutputVelocity(Constants.SnatchAndSpitConstants.LowerPIDConstants.minVel,
        Constants.SnatchAndSpitConstants.LowerPIDConstants.smartMotionSlot);
    pidControllerLower.setSmartMotionMaxAccel(Constants.SnatchAndSpitConstants.LowerPIDConstants.maxAcc,
        Constants.SnatchAndSpitConstants.LowerPIDConstants.smartMotionSlot);
    pidControllerLower.setSmartMotionAllowedClosedLoopError(
        Constants.SnatchAndSpitConstants.LowerPIDConstants.allowedError,
        Constants.SnatchAndSpitConstants.LowerPIDConstants.smartMotionSlot);

    pidControllerUpper.setP(Constants.SnatchAndSpitConstants.UpperPIDConstants.kP);
    pidControllerUpper.setI(Constants.SnatchAndSpitConstants.UpperPIDConstants.kI);
    pidControllerUpper.setD(Constants.SnatchAndSpitConstants.UpperPIDConstants.kD);
    pidControllerUpper.setIZone(Constants.SnatchAndSpitConstants.UpperPIDConstants.kIz);
    pidControllerUpper.setFF(Constants.SnatchAndSpitConstants.UpperPIDConstants.kFF);
    pidControllerUpper.setOutputRange(Constants.SnatchAndSpitConstants.UpperPIDConstants.kMinOutput,
        Constants.SnatchAndSpitConstants.UpperPIDConstants.kMaxOutput);
    pidControllerUpper.setSmartMotionMaxVelocity(Constants.SnatchAndSpitConstants.UpperPIDConstants.maxVel,
        Constants.SnatchAndSpitConstants.UpperPIDConstants.smartMotionSlot);
    pidControllerUpper.setSmartMotionMinOutputVelocity(Constants.SnatchAndSpitConstants.UpperPIDConstants.minVel,
        Constants.SnatchAndSpitConstants.UpperPIDConstants.smartMotionSlot);
    pidControllerUpper.setSmartMotionMaxAccel(Constants.SnatchAndSpitConstants.UpperPIDConstants.maxAcc,
        Constants.SnatchAndSpitConstants.UpperPIDConstants.smartMotionSlot);
    pidControllerUpper.setSmartMotionAllowedClosedLoopError(
        Constants.SnatchAndSpitConstants.UpperPIDConstants.allowedError,
        Constants.SnatchAndSpitConstants.UpperPIDConstants.smartMotionSlot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    pidControllerLower.setReference(TargetVelocityLower, CANSparkMax.ControlType.kVelocity);
    pidControllerUpper.setReference(TargetVelocityUpper, CANSparkMax.ControlType.kVelocity);

    SmartDashboard.getBoolean("Intake Velocity", currentRPMIndex == 0);
    SmartDashboard.getBoolean("High Velocity", currentRPMIndex == 1);
    SmartDashboard.getBoolean("Middle Velocity", currentRPMIndex == 2);
    SmartDashboard.getBoolean("Drop Velocity", currentRPMIndex == 3);

  }

  public void stop() {
    TargetVelocityLower = 0;
    TargetVelocityUpper = 0;
  }

  private double calculateVelocity() {
    // TODO: create equation to calculate velocity based on distance
    return 0.0;
  }

  public void intake() {
    // TODO: turn wheels so cube is injested

    TargetVelocityLower = Constants.SnatchAndSpitConstants.VelocityConstants.Lower.intake;
    TargetVelocityUpper = Constants.SnatchAndSpitConstants.VelocityConstants.Upper.intake;
    currentRPMIndex = 0;
  }

  public void shootHigh() {
    // TODO: turn wheels so cube is shot at high shelf
    TargetVelocityLower = Constants.SnatchAndSpitConstants.VelocityConstants.Lower.shootHigh;
    TargetVelocityUpper = Constants.SnatchAndSpitConstants.VelocityConstants.Upper.shootHigh;
    currentRPMIndex = 1;

  }

  public void shootMiddle() {
    // TODO: turn wheels so cube is shot at middle shelf
    TargetVelocityLower = Constants.SnatchAndSpitConstants.VelocityConstants.Lower.shootMiddle;
    TargetVelocityUpper = Constants.SnatchAndSpitConstants.VelocityConstants.Upper.shootMiddle;
    currentRPMIndex = 2;

  }

  public void drop() {
    // TODO: turn wheels so cube is dropped into hybrid
    TargetVelocityLower = Constants.SnatchAndSpitConstants.VelocityConstants.Lower.drop;
    TargetVelocityUpper = Constants.SnatchAndSpitConstants.VelocityConstants.Upper.drop;
    currentRPMIndex = 3;

  }

  public void stepUp() {
    if (currentRPMIndex < 3) {
      currentRPMIndex++;
    }
    TargetVelocityLower = Constants.SnatchAndSpitConstants.VelocityConstants.Lower.validVelocity[currentRPMIndex];
    TargetVelocityUpper = Constants.SnatchAndSpitConstants.VelocityConstants.Upper.validVelocity[currentRPMIndex];

  }

  public void stepDown() {
    if (currentRPMIndex < 0) {
      currentRPMIndex--;
    }
    TargetVelocityLower = Constants.SnatchAndSpitConstants.VelocityConstants.Lower.validVelocity[currentRPMIndex];
    TargetVelocityUpper = Constants.SnatchAndSpitConstants.VelocityConstants.Upper.validVelocity[currentRPMIndex];

  }

}
