package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;


public class ArmGoDown extends SequentialCommandGroup{
    
    public ArmGoDown(ArmSubsystem arm){
        addCommands(
            new RunCommand(() -> arm.gotoArmAngle(Constants.DOWN_POSITION), arm)
        );
           
    }
}
