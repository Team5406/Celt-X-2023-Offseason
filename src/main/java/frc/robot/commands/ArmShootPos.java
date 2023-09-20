package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;


public class ArmShootPos extends SequentialCommandGroup{
    
    public ArmShootPos(ArmSubsystem arm){
        addCommands(
            new RunCommand(() -> arm.gotoArmAngle(Constants.SHOOT_L2_POSITION), arm)
        );
           
    }
}
