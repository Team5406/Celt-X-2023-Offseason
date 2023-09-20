package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;

public class IntakeSpin extends SequentialCommandGroup{
    
    public IntakeSpin(IntakeSubsystem intake, double speed){
        addCommands(
            new RunCommand(() -> intake.cubeIntake(speed), intake)
           // new RunCommand(() -> roller.cubeIntake(Constants.TOP_ROLLER_SPEED_CUBE), roller)
        );
           
    }
}