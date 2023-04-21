// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase {

  private final RelativeEncoder encoder;
  private SparkMaxPIDController pidController;
  private final CANSparkMax SparkMax = new CANSparkMax(Constants.ElbowConstants.elbowSparkMaxID,
      MotorType.kBrushed);

  /** Creates a new Elbow. */
  public Elbow() {

    SparkMax.restoreFactoryDefaults();
    SparkMax.setIdleMode(IdleMode.kBrake);
    SparkMax.enableVoltageCompensation(12);

    encoder = SparkMax.getEncoder();

    pidController = SparkMax.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(Constants.ElbowConstants.ElbowPIDConstants.kP);
    pidController.setI(Constants.ElbowConstants.ElbowPIDConstants.kI);
    pidController.setD(Constants.ElbowConstants.ElbowPIDConstants.kD);
    pidController.setIZone(Constants.ElbowConstants.ElbowPIDConstants.kIz);
    pidController.setFF(Constants.ElbowConstants.ElbowPIDConstants.kFF);
    pidController.setOutputRange(Constants.ElbowConstants.ElbowPIDConstants.kMinOutput,
        Constants.ElbowConstants.ElbowPIDConstants.kMaxOutput);
    pidController.setSmartMotionMaxVelocity(Constants.ElbowConstants.ElbowPIDConstants.maxVel,
        Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(Constants.ElbowConstants.ElbowPIDConstants.minVel,
        Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);
    pidController.setSmartMotionMaxAccel(Constants.ElbowConstants.ElbowPIDConstants.maxAcc,
        Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(
        Constants.ElbowConstants.ElbowPIDConstants.allowedError,
        Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
