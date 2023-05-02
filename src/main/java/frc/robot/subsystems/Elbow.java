// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase {

  private final RelativeEncoder encoder;
  private SparkMaxPIDController pidController;
  private final CANSparkMax SparkMaxRight = new CANSparkMax(Constants.ElbowConstants.elbowSparkMaxRightID,MotorType.kBrushless);
  private final CANSparkMax SparkMaxLeft = new CANSparkMax(Constants.ElbowConstants.elbowSparkMaxLeftID, MotorType.kBrushless) ;

  private int currentPositionIndex = 0;

  /** Creates a new Elbow. */
  public Elbow() {

    SparkMaxRight.restoreFactoryDefaults();
    SparkMaxRight.setIdleMode(IdleMode.kBrake);
    SparkMaxRight.enableVoltageCompensation(12);
    
    SparkMaxLeft.restoreFactoryDefaults();
    SparkMaxLeft.setIdleMode(IdleMode.kBrake);
    SparkMaxLeft.enableVoltageCompensation(12);

    encoder = SparkMaxRight.getEncoder();

    pidController = SparkMaxRight.getPIDController();
    pidController.setFeedbackDevice(encoder);

    pidController.setP(Constants.ElbowConstants.ElbowPIDConstants.kP);
    pidController.setI(Constants.ElbowConstants.ElbowPIDConstants.kI);
    pidController.setD(Constants.ElbowConstants.ElbowPIDConstants.kD);
    pidController.setIZone(Constants.ElbowConstants.ElbowPIDConstants.kIz);
    pidController.setFF(Constants.ElbowConstants.ElbowPIDConstants.kFF);
    pidController.setOutputRange(Constants.ElbowConstants.ElbowPIDConstants.kMinOutput, Constants.ElbowConstants.ElbowPIDConstants.kMaxOutput);
    pidController.setSmartMotionMaxVelocity(Constants.ElbowConstants.ElbowPIDConstants.maxVel, Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(Constants.ElbowConstants.ElbowPIDConstants.minVel, Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);
    pidController.setSmartMotionMaxAccel(Constants.ElbowConstants.ElbowPIDConstants.maxAcc, Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(Constants.ElbowConstants.ElbowPIDConstants.allowedError, Constants.ElbowConstants.ElbowPIDConstants.smartMotionSlot);

    SparkMaxLeft.follow(SparkMaxRight, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Start", currentPositionIndex == 0);
    SmartDashboard.putBoolean("Shoot", currentPositionIndex == 1);
    SmartDashboard.putBoolean("Drop", currentPositionIndex == 2);
    SmartDashboard.putBoolean("Intake", currentPositionIndex == 3);
  }

  public void stepUp() {
    if (currentPositionIndex < 3) {
      currentPositionIndex++;
    }
  }

  public void stepDown() {
    if (currentPositionIndex > 0) {
      currentPositionIndex--;
    }
  }

  public void setElbowPosition(){
    pidController.setReference(Constants.ElbowConstants.ElbowPositionConstants.positions[currentPositionIndex], ControlType.kPosition, 0, 0.00, ArbFFUnits.kPercentOut);
  }

}
