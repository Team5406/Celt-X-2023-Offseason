// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.ArmGoDown;
import frc.robot.commands.ArmGoUp;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.GoToAngle;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.OuttakeCube;
import frc.robot.commands.ShootL2;
import frc.robot.commands.ShootL3;
import frc.robot.commands.ShootL3Backwards;
import frc.robot.commands.StopIntake;
import frc.robot.commands.StopRoller;
import frc.robot.commands.ShootL2Backwards;
import frc.robot.commands.OuttakeCubeBackwards;
import frc.robot.commands.RollerSpin;
import frc.robot.commands.IntakeSpin;
import frc.robot.commands.MoveArmTrapezoid;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;

import java.io.File;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TopRollerSubsystem;

import frc.robot.autos.DriveStraight;
import frc.robot.autos.DoNothing;
import frc.robot.autos.Balance;
import frc.robot.autos.L2CubeMobilityBalance;
import frc.robot.autos.L3Mobility;
import frc.robot.autos.L3MobilityBump;
import frc.robot.autos.CubeCubeCleanBlue;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  IntakeSubsystem intake = new IntakeSubsystem();
  TopRollerSubsystem roller = new TopRollerSubsystem();
  ArmSubsystem arm = new ArmSubsystem();
  CommandBase driveStraight = DriveStraight.auto(swerve);
  CommandBase doNothing = DoNothing.auto();
  CommandBase balance = Balance.auto(swerve);
  CommandBase l2CubeMobilityBalance = L2CubeMobilityBalance.auto(swerve, intake, roller, arm);
  CommandBase cubeCubeCleanBlue = CubeCubeCleanBlue.auto(swerve, arm, intake, roller);
  CommandBase l3Mobility = L3Mobility.auto(swerve, arm, intake, roller);
  CommandBase l3MobilityBump = L3MobilityBump.auto(swerve, arm, intake, roller);

 
  // Replace with CommandPS4Controller or CommandJoystick if needed
  CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER);

  CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER);

  DigitalInput m_limitSwitch = new DigitalInput(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    TeleopDrive driveCommand = new TeleopDrive(
    swerve,
    () -> aboveDeadband(-driverController.getLeftY()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
    () -> aboveDeadband(-driverController.getLeftX()/inputScale(driverController.getLeftX(), driverController.getLeftY())),
    () -> -driverController.getRightX(), () -> true, true, false);

  swerve.setDefaultCommand(driveCommand);

  intake.setDefaultCommand(new StopIntake(intake));
  roller.setDefaultCommand(new StopRoller(roller));
  arm.setDefaultCommand(new ArmGoUp(arm));

  m_chooser.setDefaultOption("1 - Drive Straight Auto", driveStraight);
  m_chooser.addOption("1 - Do Nothing", doNothing);
  m_chooser.addOption("3 - Balance", balance);
  m_chooser.addOption("4 - L2CubeMobBal", l2CubeMobilityBalance);
  m_chooser.addOption("5 - CubeCubeCLeanBlue", cubeCubeCleanBlue);
  m_chooser.addOption("6 - L3Mobility", l3Mobility);
  m_chooser.addOption("7 - L3MobilityBump", l3MobilityBump);
  SmartDashboard.putData(m_chooser);

  }
  SendableChooser<Command> m_chooser = new SendableChooser<>();
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    driverController.back().onTrue((new InstantCommand(swerve::zeroGyro)));

    driverController.x().whileTrue(new RepeatCommand(new InstantCommand(swerve::lock, swerve))); //x lock

    driverController.rightTrigger()
    .whileTrue(new IntakeCube(intake, arm, roller));

    driverController.a()
    .whileTrue(new ShootL2(intake, arm, roller));

    driverController.povDown()
    .whileTrue(new ShootL2Backwards(intake, arm, roller));

    driverController.povRight()
    .whileTrue(new ShootL3Backwards(intake, arm, roller));

    driverController.b()
    .whileTrue(new ShootL3(intake, arm, roller));

    driverController.rightBumper().whileTrue(new IntakeSpin(intake, Constants.INTAKE_SPEED_CUBE));

    driverController.leftTrigger().whileTrue(new OuttakeCube(intake, arm));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public double inputScale(double x, double y) {
    double xAbs = Math.abs(x);
    double yAbs = Math.abs(y);
    double angle = Math.atan2(yAbs, xAbs);
    double result = yAbs > xAbs ? Math.sin(angle) : Math.cos(angle);
    return result;
}

   public double aboveDeadband(double input) {
    return (Math.abs(input) > Constants.DEFAULT_DEADBAND) ? input : 0;
}

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
