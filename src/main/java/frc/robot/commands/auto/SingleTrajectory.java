package frc.robot.commands.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elbow;
import frc.robot.subsystems.SnatchAndSpit;
import frc.robot.Constants.AutoDriveConstants;

public class SingleTrajectory extends SequentialCommandGroup {

  private DriveBase m_drivetrain;
  private Trajectory m_trajectory;
  private SnatchAndSpit m_snatchAndSpit;
  private Elbow m_elbow;

  public SingleTrajectory(SnatchAndSpit in_snatchAndSpit, Elbow in_elbow, DriveBase in_drivetrain,
      Trajectory in_trajectory, String route) {

    m_drivetrain = in_drivetrain;
    m_trajectory = in_trajectory;
    m_snatchAndSpit = in_snatchAndSpit;
    m_elbow = in_elbow;

    var leftController = new PIDController(AutoDriveConstants.kPDriveVel, 0, 0);
    var rightController = new PIDController(AutoDriveConstants.kPDriveVel, 0, 0);

    var table = NetworkTableInstance.getDefault().getTable("troubleshooting");
    var leftReference = table.getEntry("left_reference");
    var leftMeasurement = table.getEntry("left_measurement");
    var rightReference = table.getEntry("right_reference");
    var rightMeasurement = table.getEntry("right_measurement");

    RamseteCommand ramseteCommand = new RamseteCommand(
        m_trajectory,
        m_drivetrain::getPose,
        new RamseteController(AutoDriveConstants.kRamseteB, AutoDriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            AutoDriveConstants.ksVolts,
            AutoDriveConstants.kvVoltSecondsPerMeter,
            AutoDriveConstants.kaVoltSecondsSquaredPerMeter),
        AutoDriveConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        leftController,
        rightController,
        // RamseteCommand passes volts to the callback
        (leftVolts, rightVolts) -> {
          m_drivetrain.tankDriveVolts(leftVolts, rightVolts);

          leftMeasurement.setNumber(m_drivetrain.getWheelSpeeds().leftMetersPerSecond);
          leftReference.setNumber(leftController.getSetpoint());

          rightMeasurement.setNumber(m_drivetrain.getWheelSpeeds().rightMetersPerSecond);
          rightReference.setNumber(rightController.getSetpoint());
        },
        m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(m_trajectory.getInitialPose());

    switch (route) {
      case "balance":
        balanceRoutine(ramseteCommand);
        break;
      case "side":
        sideRoutine(ramseteCommand);
        break;
      case "center":
        centerRoutine(ramseteCommand);
        break;
      case "move":
        moveRoutine(ramseteCommand);
        break;
      default:
        moveRoutine(ramseteCommand);
    }

  }

  private void moveRoutine(RamseteCommand ramseteCommand) {

    addCommands(
      new InstantCommand(m_snatchAndSpit::spitFast,m_snatchAndSpit),
      new WaitCommand(1),
      new InstantCommand(m_snatchAndSpit::end,m_snatchAndSpit),
      ramseteCommand.andThen(new InstantCommand(m_drivetrain::setBrake, m_drivetrain).andThen(new InstantCommand(m_drivetrain::tankDriveVoltageStop, m_drivetrain)))
    );
  }

  private void balanceRoutine(RamseteCommand ramseteCommand) {

    addCommands(
      new InstantCommand(m_snatchAndSpit::spitFast,m_snatchAndSpit),
      new WaitCommand(1),
      new InstantCommand(m_snatchAndSpit::end,m_snatchAndSpit),
      ramseteCommand.andThen(new InstantCommand(m_drivetrain::setBrake, m_drivetrain).andThen(new InstantCommand(m_drivetrain::tankDriveVoltageStop, m_drivetrain))),
      new InstantCommand(m_drivetrain::autoBalanceWithPID,m_drivetrain)
    );
  }

  private void sideRoutine(RamseteCommand ramseteCommand) {

    addCommands(
      new InstantCommand(m_snatchAndSpit::spitFast,m_snatchAndSpit),
      new WaitCommand(1),
      new InstantCommand(m_snatchAndSpit::end,m_snatchAndSpit),
      new ParallelCommandGroup(      
        ramseteCommand.andThen(new InstantCommand(m_drivetrain::setBrake, m_drivetrain).andThen(new InstantCommand(m_drivetrain::tankDriveVoltageStop, m_drivetrain))),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new InstantCommand(m_elbow::intake,m_elbow),
          new WaitCommand(5),
          new InstantCommand(m_elbow::Start,m_elbow)
        ),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new InstantCommand(m_snatchAndSpit::intake,m_snatchAndSpit),
          new WaitCommand(5),
          new InstantCommand(m_snatchAndSpit::intake,m_snatchAndSpit)
        )
      ),
      new InstantCommand(m_snatchAndSpit::spitSlow,m_snatchAndSpit)
    );
  }

  private void centerRoutine(RamseteCommand ramseteCommand) {

    addCommands(
      new InstantCommand(m_snatchAndSpit::spitFast,m_snatchAndSpit),
      new WaitCommand(1),
      new InstantCommand(m_snatchAndSpit::end,m_snatchAndSpit),
      new ParallelCommandGroup(      
        ramseteCommand.andThen(new InstantCommand(m_drivetrain::setBrake, m_drivetrain).andThen(new InstantCommand(m_drivetrain::tankDriveVoltageStop, m_drivetrain))),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new InstantCommand(m_elbow::intake,m_elbow),
          new WaitCommand(5),
          new InstantCommand(m_elbow::Start,m_elbow)
        ),
        new SequentialCommandGroup(
          new WaitCommand(1.5),
          new InstantCommand(m_snatchAndSpit::intake,m_snatchAndSpit),
          new WaitCommand(5),
          new InstantCommand(m_snatchAndSpit::intake,m_snatchAndSpit)
        )
      ),
      new InstantCommand(m_snatchAndSpit::spitSlow,m_snatchAndSpit)
    );
  }
}
