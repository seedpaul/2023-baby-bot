// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.auto.SingleTrajectory;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.SnatchAndSpit;
import frc.robot.subsystems.components.LED;
import frc.robot.subsystems.components.NavX;
import frc.robot.subsystems.components.TriggerButton;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final NavX navX = new NavX();
  private final DriveBase babyBotBase = new DriveBase(navX);
  private LED led = new LED(this);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final XboxController assistController = new XboxController(OperatorConstants.kAssistControllerPort);

  private final SnatchAndSpit snatchAndSpitSubsystem = SnatchAndSpit.getInstance();
  private final Elbow elbow = Elbow.getInstance();
  
  private SingleTrajectory move,balance,side,center;
  private SendableChooser<Command> m_chooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    Trajectory moveTrajectory = PathPlanner.loadPath("MovePath", new PathConstraints(.85, 1.0));
    Trajectory balanceTrajectory = PathPlanner.loadPath("BalancePath", new PathConstraints(.85, 1.0));
    Trajectory sideTrajectory = PathPlanner.loadPath("SideCubePath", new PathConstraints(.85, 1.0));
    Trajectory centerTrajectory = PathPlanner.loadPath("CenterCubePath", new PathConstraints(.85, 1.0));

    move = new SingleTrajectory(snatchAndSpitSubsystem,elbow,babyBotBase,moveTrajectory,"move");
    balance = new SingleTrajectory(snatchAndSpitSubsystem,elbow,babyBotBase,balanceTrajectory,"balance");
    side = new SingleTrajectory(snatchAndSpitSubsystem,elbow,babyBotBase,sideTrajectory,"side");
    center = new SingleTrajectory(snatchAndSpitSubsystem,elbow,babyBotBase,centerTrajectory,"center");

    this.m_chooser = new SendableChooser<Command>();
    this.m_chooser.setDefaultOption("Move", move);
    this.m_chooser.addOption("Balance", balance);
    this.m_chooser.addOption("Side", side);
    this.m_chooser.addOption("Center", center);

    // Put the chooser on the dashboard
    SmartDashboard.putData(this.m_chooser);

    babyBotBase.setDefaultCommand(
        // rotation speed
        new RunCommand(() -> babyBotBase.arcadeDrive(driverController.getLeftY(), driverController.getRightX()),
            babyBotBase));
  }

  private void configureBindings() {

    TriggerButton rightTrigger_as = new TriggerButton(assistController, XboxController.Axis.kRightTrigger);
    TriggerButton leftTrigger_as = new TriggerButton(assistController, XboxController.Axis.kLeftTrigger);

    JoystickButton buttonB_dr = new JoystickButton(driverController, Constants.OperatorConstants.B_BUTTON);
    JoystickButton buttonX_dr = new JoystickButton(driverController, Constants.OperatorConstants.X_BUTTON);

    JoystickButton buttonY_as = new JoystickButton(assistController, Constants.OperatorConstants.Y_BUTTON);
    JoystickButton buttonA_as = new JoystickButton(assistController, Constants.OperatorConstants.A_BUTTON);

    JoystickButton rightBumper_as = new JoystickButton(assistController, Constants.OperatorConstants.RIGHT_BUMPER);
    JoystickButton leftBumper_as = new JoystickButton(assistController, Constants.OperatorConstants.LEFT_BUMPER);

    //driver *******************************************************************
    //manual override elbow
    buttonB_dr.whileTrue(new InstantCommand(elbow::up, elbow));
    buttonB_dr.onFalse(new InstantCommand(elbow::stop, elbow));

    buttonX_dr.whileTrue(new InstantCommand(elbow::down, elbow));
    buttonX_dr.onFalse(new InstantCommand(elbow::stop, elbow));
    //driver end *******************************************************************


    //assistant*******************************************************************
    //step up and down
    buttonY_as.whileTrue(new InstantCommand(elbow::stepUp, elbow));
    buttonA_as.whileTrue(new InstantCommand(elbow::stepDown, elbow));

    //eject
    rightBumper_as.whileTrue(new InstantCommand(snatchAndSpitSubsystem::spitFast, snatchAndSpitSubsystem));
    rightBumper_as.onFalse(new InstantCommand(snatchAndSpitSubsystem::end, snatchAndSpitSubsystem));

    rightTrigger_as.whileTrue(new InstantCommand(snatchAndSpitSubsystem::spitSlow, snatchAndSpitSubsystem));
    rightTrigger_as.onFalse(new InstantCommand(snatchAndSpitSubsystem::end, snatchAndSpitSubsystem));

    //intake
    leftBumper_as.onTrue(new InstantCommand(snatchAndSpitSubsystem::intake, snatchAndSpitSubsystem));
    leftTrigger_as.onTrue(new InstantCommand(snatchAndSpitSubsystem::end, snatchAndSpitSubsystem));
    //end assistant*******************************************************************
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }

  public boolean isRedAlliance() {
    return DriverStation.getAlliance() == Alliance.Red;
  }

  public boolean isDisabled() {
    return DriverStation.isDisabled();
  }

  public boolean isAutonomous() {
    return DriverStation.isAutonomous();
  }

  public boolean isTeleop() {
    return DriverStation.isTeleop();
  }
}

