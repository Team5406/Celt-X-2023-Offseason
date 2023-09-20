package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.Constants;


public class RollerSpin extends SequentialCommandGroup{
    
    public RollerSpin(TopRollerSubsystem roller){
        addCommands(
            new RunCommand(() -> roller.cubeIntake(Constants.TOP_ROLLER_SPEED_CUBE), roller)
        );
           
    }
}
