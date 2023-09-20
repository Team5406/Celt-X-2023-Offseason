package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.SwerveSubsystem;

public final class Balance
{

  /**
   * April Tag field layout.
   */

  private Balance()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Example static factory for an autonomous command.
   */
  public static CommandBase auto(SwerveSubsystem swerve)
  {
   
    return Commands.sequence(
      new AutoBalance(swerve, false)
      //new RepeatCommand(new InstantCommand(swerve::lock, swerve))
    );
  }

 
}