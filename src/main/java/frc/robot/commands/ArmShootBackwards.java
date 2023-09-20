package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;


public class ArmShootBackwards extends SequentialCommandGroup{
    
    public ArmShootBackwards(ArmSubsystem arm){
        addCommands(
            new RunCommand(() -> arm.gotoArmAngle(Constants.SHOOT_L2_BACKWARDS_POSITION), arm)
        );
           
    }
}
