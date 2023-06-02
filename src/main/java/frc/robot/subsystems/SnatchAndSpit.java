// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SnatchAndSpit extends SubsystemBase {

  private static SnatchAndSpit sas = null;

  private final CANSparkMax lowerSparkMax = new CANSparkMax(Constants.SnatchAndSpitConstants.lowerSparkMaxID,
      MotorType.kBrushless);
  private final CANSparkMax upperSparkMax = new CANSparkMax(Constants.SnatchAndSpitConstants.upperSparkMaxID,
      MotorType.kBrushless);

  private boolean fastIntake = false;

  public static SnatchAndSpit getInstance() {
    if (sas == null) {
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

    lowerSparkMax.setSmartCurrentLimit(10, 80,50);
    upperSparkMax.setSmartCurrentLimit(10, 80,50);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LowerOutputCurrent", lowerSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("LowerAppliedOutput", lowerSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("LowerBusVoltage", lowerSparkMax.getBusVoltage());

    SmartDashboard.putNumber("UpperOutputCurrent", upperSparkMax.getOutputCurrent());
    SmartDashboard.putNumber("UpperAppliedOutput", upperSparkMax.getAppliedOutput());
    SmartDashboard.putNumber("UpperBusVoltage", upperSparkMax.getBusVoltage());
  }

  public void spitFast() {
    lowerSparkMax.set(-1);
    upperSparkMax.set(-1);
  }

  public void spitSlow() {
    lowerSparkMax.set(-0.25);
    upperSparkMax.set(-0.25);
  }

  public void intake() {

    fastIntake = !fastIntake;

    if (fastIntake) {
      lowerSparkMax.set(0.2);
      upperSparkMax.set(0.2);
    } else {
      lowerSparkMax.set(0.03);
      upperSparkMax.set(0.03);
    }
  }

  public void end() {
    lowerSparkMax.set(0.0);
    upperSparkMax.set(0.0);
  }

}
