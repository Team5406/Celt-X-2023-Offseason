package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;

public final class CubeCubeBumpBlue
{

  /**
   * April Tag field layout.
   */
  private CubeCubeBumpBlue()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake, TopRollerSubsystem roller)
  {
    boolean blue = true;
    double multiplier = blue?-1:1;

    PathPlannerTrajectory driveToCube;
    PathPlannerTrajectory driveToGrid;
    PathPlannerTrajectory driveToCube2;
    PathPlannerTrajectory driveBack;
      // Simple path with holonomic rotation. Stationary start/end. Max velocity of 4 m/s and max accel of 3 m/s^2
      driveToCube = PathPlanner.generatePath(
        new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
        new PathPoint(new Translation2d(0, multiplier*0), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*0)),
        new PathPoint(new Translation2d(-2.7, multiplier*0.05), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*-5)),
        new PathPoint(new Translation2d(-4.0, multiplier*-0.25), Rotation2d.fromDegrees(180), Rotation2d.fromDegrees(multiplier*178))
    );
    driveToGrid = PathPlanner.generatePath(
      new PathConstraints(Constants.MAX_SPEED_METERS_PER_SECOND, Constants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED),
      true,
      new PathPoint(new Translation2d(-3, multiplier*0.01), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*5)),
      new PathPoint(new Translation2d(-0.15, multiplier*0.9), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*0)),
      new PathPoint(new Translation2d(0.1, multiplier*0.9), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(multiplier*0))
  );



      // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want
      // to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
      //swerve.postTrajectory(driveToCube);
      //swerve.postTrajectory(driveToGrid);
      return Commands.sequence(new SequentialCommandGroup(
      new InstantCommand(() -> swerve.resetOdometry(driveToCube.getInitialHolonomicPose())),
      /*new ParallelRaceGroup(
        new WaitCommand(1.75),
        new ShootL2(intake, arm, roller)
      ),*/
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FollowTrajectory(swerve, driveToCube, true)
        )/* 
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(2),
            new OuttakeCube(intake, arm, roller)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new IntakeCube(intake, arm, roller)
          )*/
        //)
      ),
      //new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)), 
      new ParallelCommandGroup(
        new FollowTrajectory(swerve, driveToGrid, false),
        new SequentialCommandGroup(
          new WaitCommand(2)
          /* ParallelRaceGroup(
            new WaitCommand(1.5),
            new OuttakeCube(intake, arm, roller)
          )*/
        )
      ),
      /*
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new WaitCommand(1),
          new FollowTrajectory(swerve, driveToCube2, false)
        ) ,
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new WaitCommand(2),
            new OuttakeCube(intake, arm, roller)
          ),
          new WaitCommand(0.5),
          new ParallelDeadlineGroup(
            new WaitCommand(1),
            new IntakeCube(intake, arm, roller)
          )
        )*/
      //new InstantCommand(() -> intake.setIntakeVoltage(Constants.INTAKE_VOLTAGE_HOLDING_CUBE)),
      //new InstantCommand(() -> wrist.gotoWristAngle(Constants.WRIST_HOME_POSITION), intake), 
      //new InstantCommand(() -> intake.setGamePieceType(GamePieces.CONE), intake), 
      new WaitCommand(0.5),
      new RepeatCommand(new InstantCommand(swerve::lock, swerve))
    ));
  }

 
}
