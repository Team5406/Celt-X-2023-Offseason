package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;
import frc.robot.Constants;


public class ShootL2Backwards extends SequentialCommandGroup{
    
    public ShootL2Backwards(IntakeSubsystem intake, ArmSubsystem arm, TopRollerSubsystem roller){
        addCommands(
            new GoToAngle(arm, Constants.SHOOT_L2_BACKWARDS_POSITION),
            new RunCommand(() -> intake.cubeShoot(Constants.SHOOT_L2_BACKWARDS_SPEED), intake)
        );
           
    }

}
