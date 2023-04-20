// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class DriveBase extends SubsystemBase {

  private final WPI_TalonFX leftFrontTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.leftFrontTalonFX_ID);
  private final WPI_TalonFX leftBackTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.leftBackTalonFX_ID);
  private final WPI_TalonFX rightFrontTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.rightFrontTalonFX_ID);
  private final WPI_TalonFX rightBackTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.rightBackTalonFX_ID);

  private final MotorControllerGroup leftSideMotorControllerGroup = new MotorControllerGroup(leftFrontTalonFX, leftBackTalonFX);
  private final MotorControllerGroup rightSideMotorControllerGroup = new MotorControllerGroup(rightFrontTalonFX, rightBackTalonFX);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftSideMotorControllerGroup, rightSideMotorControllerGroup);

  /** Creates a new DriveBase. */
  public DriveBase() {
    // this is a constructor, this is called when we create an instance of the drive base
    System.out.println("drive train created");

    leftFrontTalonFX.setNeutralMode(NeutralMode.Coast);
    leftBackTalonFX.setNeutralMode(NeutralMode.Coast);
    rightFrontTalonFX.setNeutralMode(NeutralMode.Coast);
    rightBackTalonFX.setNeutralMode(NeutralMode.Coast);

    SupplyCurrentLimitConfiguration SCLC = new SupplyCurrentLimitConfiguration(true,40,60,1);
    leftFrontTalonFX.configSupplyCurrentLimit(SCLC);
    leftBackTalonFX.configSupplyCurrentLimit(SCLC);
    rightFrontTalonFX.configSupplyCurrentLimit(SCLC);
    rightBackTalonFX.configSupplyCurrentLimit(SCLC);

    driveTrain.setExpiration(1);
    driveTrain.setSafetyEnabled(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void arcadeDrive(double rotation, double speed){

    double modifiedSpeed = speed * Constants.DriveBaseConstants.speedGoverner;
    double modifiedRotation = rotation * Constants.DriveBaseConstants.rotationGoverner;

    this.driveTrain.arcadeDrive(modifiedSpeed, modifiedRotation);
  }
}
