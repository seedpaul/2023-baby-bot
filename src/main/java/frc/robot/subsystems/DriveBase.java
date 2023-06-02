// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;
import frc.robot.subsystems.components.NavX;

public class DriveBase extends SubsystemBase {

  private final WPI_TalonFX leftFrontTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.leftFrontTalonFX_ID);
  private final WPI_TalonFX leftBackTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.leftBackTalonFX_ID);
  private final WPI_TalonFX rightFrontTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.rightFrontTalonFX_ID);
  private final WPI_TalonFX rightBackTalonFX = new WPI_TalonFX(Constants.DriveBaseConstants.rightBackTalonFX_ID);

  private final MotorControllerGroup leftSideMotorControllerGroup = new MotorControllerGroup(leftFrontTalonFX,leftBackTalonFX);
  private final MotorControllerGroup rightSideMotorControllerGroup = new MotorControllerGroup(rightFrontTalonFX,rightBackTalonFX);

  private final DifferentialDrive driveTrain = new DifferentialDrive(leftSideMotorControllerGroup,rightSideMotorControllerGroup);

  private final DifferentialDriveOdometry m_odometry;

    // Network tables for odometry logging
    private NetworkTableEntry m_xEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("X");
    private NetworkTableEntry m_yEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Y");
    private NetworkTableEntry m_LeftEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Left");
    private NetworkTableEntry m_RightEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Right");
    private NetworkTableEntry m_RotEntry = NetworkTableInstance.getDefault().getTable("troubleshooting").getEntry("Rotation");

  private NavX m_NavX;

  private final double Kp = 0.0, Ki = 0.0, Kd = 0.0;
  private final PIDController balancePIDController = new PIDController(Kp, Ki, Kd);
  private int counter = 0;

  /** Creates a new DriveBase. */
  public DriveBase(NavX in_navX) {
    // this is a constructor, this is called when we create an instance of the drive
    // base
    System.out.println("drive train created");

    m_NavX = in_navX;

    balancePIDController.setTolerance(Constants.DriveBaseConstants.balanceRollThreshold);

    leftFrontTalonFX.setNeutralMode(NeutralMode.Coast);
    leftBackTalonFX.setNeutralMode(NeutralMode.Coast);
    rightFrontTalonFX.setNeutralMode(NeutralMode.Coast);
    rightBackTalonFX.setNeutralMode(NeutralMode.Coast);

    SupplyCurrentLimitConfiguration SCLC = new SupplyCurrentLimitConfiguration(true, 40, 60, 1);
    leftFrontTalonFX.configSupplyCurrentLimit(SCLC);
    leftBackTalonFX.configSupplyCurrentLimit(SCLC);
    rightFrontTalonFX.configSupplyCurrentLimit(SCLC);
    rightBackTalonFX.configSupplyCurrentLimit(SCLC);

    driveTrain.setExpiration(1);
    driveTrain.setSafetyEnabled(false);

    m_odometry = new DifferentialDriveOdometry(m_NavX.getRotation2d(), 0, 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isAutonomous()) {
      // Update the odometry in the periodic block
      m_odometry.update(m_NavX.getRotation2d(), this.getLeftDistance(), this.getRightDistance());

      if (counter % 100 == 0) {
        var translation = m_odometry.getPoseMeters().getTranslation();
        m_xEntry.setNumber(translation.getX());
        m_yEntry.setNumber(translation.getY());

        m_LeftEntry.setNumber(this.getLeftDistance());
        m_RightEntry.setNumber(this.getRightDistance());

        Rotation2d rot = m_NavX.getRotation2d();
        m_RotEntry.setNumber(rot.getDegrees());
      }
    } else {
      leftFrontTalonFX.setNeutralMode(NeutralMode.Brake);
      rightFrontTalonFX.setNeutralMode(NeutralMode.Brake);
    }

    SmartDashboard.putNumber("Left Distance", getLeftDistance());
    SmartDashboard.putNumber("Right Distance", getRightDistance());
  }

  public void arcadeDrive(double rotation, double speed) {

    double modifiedSpeed = speed * Constants.DriveBaseConstants.speedGoverner;
    double modifiedRotation = rotation * Constants.DriveBaseConstants.rotationGoverner;

    driveTrain.arcadeDrive(modifiedSpeed, modifiedRotation);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {

    // raw sensor per 100ms, scale to meters per sec
    double leftRate = leftBackTalonFX.getSelectedSensorVelocity() * (10.0 / Constants.DriveBaseConstants.cpr)
        * Constants.DriveBaseConstants.wheelcircumference;
    double rightRate = rightBackTalonFX.getSelectedSensorVelocity() * (10.0 / Constants.DriveBaseConstants.cpr)
        * Constants.DriveBaseConstants.wheelcircumference;

    return new DifferentialDriveWheelSpeeds(leftRate, rightRate);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  private double getLeftDistance() {
    return -(leftBackTalonFX.getSelectedSensorPosition() * Constants.DriveBaseConstants.distancePerPulse);
  }

  private double getRightDistance() {
    return (rightBackTalonFX.getSelectedSensorPosition() * Constants.DriveBaseConstants.distancePerPulse);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftSideMotorControllerGroup.setVoltage(-leftVolts);
    rightSideMotorControllerGroup.setVoltage(rightVolts);
    driveTrain.feed();
  }

  public void tankDriveVoltageStop() {
    this.tankDriveVolts(0, 0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(m_NavX.getRotation2d(), 0, 0, pose);
  }

  public void resetEncoders() {
    leftBackTalonFX.setSelectedSensorPosition(0);
    rightBackTalonFX.setSelectedSensorPosition(0);
  }

  public void setBrake() {
    leftFrontTalonFX.setNeutralMode(NeutralMode.Brake);
    rightFrontTalonFX.setNeutralMode(NeutralMode.Brake);
  }

  public Command autoBalance() {
    return new RunCommand(this::SlowUntilLevel, this);
  }

  private void SlowUntilLevel() {

    double pitch = m_NavX.getPitch();
    double driveSpeed = 0;

    if (Math.abs(pitch) > Constants.DriveBaseConstants.balanceRollThreshold) {
      driveSpeed = Math.signum(pitch) * Constants.DriveBaseConstants.balanceSpeed;
    }

    tankDriveVolts(driveSpeed, -driveSpeed);
  }

  public Command autoBalanceWithPID() {
    return new RunCommand(this::balanceWithPID, this);
  }

  private void balanceWithPID() {

    double output = balancePIDController.calculate(m_NavX.getPitch(), 0);
    tankDriveVolts(output, -output);
  }
}
