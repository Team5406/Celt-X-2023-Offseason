package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.Constants;


public class Reset extends ParallelCommandGroup{
    
    public Reset(SwerveSubsystem swerve, ArmSubsystem arm, IntakeSubsystem intake, TopRollerSubsystem roller){
        addCommands(
            new GoToAngle(arm, Constants.REST_POSITION),
            new RunCommand(() -> intake.stop(), intake),
            new RunCommand(() -> roller.stop(), roller)
        );
           
    }
}
