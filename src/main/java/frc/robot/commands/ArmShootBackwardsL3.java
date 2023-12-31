package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;


public class ArmShootBackwardsL3 extends SequentialCommandGroup{
    
    public ArmShootBackwardsL3(ArmSubsystem arm){
        addCommands(
            new RunCommand(() -> arm.gotoArmAngle(Constants.SHOOT_L3_BACKWARDS_POSITION), arm)
        );
           
    }
}
