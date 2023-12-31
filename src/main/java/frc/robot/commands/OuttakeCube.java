package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;


public class OuttakeCube extends SequentialCommandGroup{
    
    public OuttakeCube(IntakeSubsystem intake, ArmSubsystem arm){
        addCommands(
            new GoToAngle(arm, Constants.DOWN_POSITION),
            new RunCommand(() -> intake.cubeShoot(Constants.OUTTAKE_L2_SPEED), intake)
        );
           
    }

}
