package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;

public class ArmShootPosL3 extends SequentialCommandGroup{
    
    public ArmShootPosL3(ArmSubsystem arm){
        addCommands(
            new RunCommand(() -> arm.gotoArmAngle(Constants.SHOOT_L3_POSITION), arm)
        );
           
    }
}
