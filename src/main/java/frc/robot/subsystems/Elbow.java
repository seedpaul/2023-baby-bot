// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elbow extends SubsystemBase {

  private static Elbow elbow = null;
  private final CANSparkMax SparkMaxRight = new CANSparkMax(Constants.ElbowConstants.elbowSparkMaxRightID, MotorType.kBrushless);
  private final CANSparkMax SparkMaxLeft = new CANSparkMax(Constants.ElbowConstants.elbowSparkMaxLeftID, MotorType.kBrushless);
  private final WPI_CANCoder cANCoder = new WPI_CANCoder(Constants.ElbowConstants.elbowCANCoderID);
  private CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();

  private PIDController pidController = new PIDController(Constants.ElbowConstants.ElbowPIDConstants.kP, Constants.ElbowConstants.ElbowPIDConstants.kI, Constants.ElbowConstants.ElbowPIDConstants.kD);

  private int currentPositionIndex = 0;

  public static Elbow getInstance() {
    if (elbow == null) {
      elbow = new Elbow();
    }
    return elbow;
  }

  /** Creates a new Elbow. */
  private Elbow() {

    canCoderConfiguration.magnetOffsetDegrees = Constants.ElbowConstants.ElbowPositionConstants.magnetOffsetDegrees;
    canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    canCoderConfiguration.sensorDirection = false;
    canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    canCoderConfiguration.sensorCoefficient = 360.0 / 4096.0;

    cANCoder.configAllSettings(canCoderConfiguration);

    SparkMaxRight.restoreFactoryDefaults();
    SparkMaxRight.setIdleMode(IdleMode.kBrake);
    SparkMaxRight.enableVoltageCompensation(12);
    SparkMaxRight.setSmartCurrentLimit(10, 65, 100);

    SparkMaxLeft.restoreFactoryDefaults();
    SparkMaxLeft.setIdleMode(IdleMode.kBrake);
    SparkMaxLeft.enableVoltageCompensation(12);
    SparkMaxLeft.setSmartCurrentLimit(1065, 100);

    SparkMaxLeft.follow(SparkMaxRight, true);

    double currentEncodeValue = cANCoder.getAbsolutePosition();
    if (currentEncodeValue < 35 && currentEncodeValue > 15) {
      currentPositionIndex = 0;
    } else if (currentEncodeValue < 111 && currentEncodeValue > 91) {
      currentPositionIndex = 1;
    } else if (currentEncodeValue < 147 && currentEncodeValue > 127) {
      currentPositionIndex = 2;
    } else {
      currentPositionIndex = 0;
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Start", currentPositionIndex == 0);
    // SmartDashboard.putBoolean("Shoot", currentPositionIndex == 1);
    SmartDashboard.putBoolean("Drop", currentPositionIndex == 1);
    SmartDashboard.putBoolean("Intake", currentPositionIndex == 2);

    SmartDashboard.putNumber("Elbow Target", Constants.ElbowConstants.ElbowPositionConstants.positions[currentPositionIndex]);
    SmartDashboard.putNumber("Elbow Encoder", cANCoder.getAbsolutePosition());

    SmartDashboard.putNumber("currentPositionIndex", currentPositionIndex);

    double output = -pidController.calculate(cANCoder.getAbsolutePosition(), Constants.ElbowConstants.ElbowPositionConstants.positions[currentPositionIndex]);
    SmartDashboard.putNumber("PID output", output);
    SparkMaxRight.set(output);
  }

  public void stepDown() {
    if (currentPositionIndex < 2) {
      currentPositionIndex++;
    }
  }

  public void stepUp() {
    if (currentPositionIndex > 0) {
      currentPositionIndex--;
    }
  }

  public void up() {
    SparkMaxRight.set(0.3);
  }

  public void down() {
    SparkMaxRight.set(-0.3);
  }

  public void stop() {
    SparkMaxRight.set(0.0);
  }

  public void intake(){
    currentPositionIndex = 2;
  }

  public void Start(){
    currentPositionIndex = 0;
  }

}
