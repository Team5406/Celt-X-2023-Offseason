// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int OPERATOR_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER = 1;
  }
  
  //General Constants
  public static final double ROBOT_MASS = Units.lbsToKilograms(90); // 32lbs * kg per pound
  public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS); //where center of mass is
  public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag()
  public static final double DEFAULT_DEADBAND = 0.05;
  public static final int DEGREES_PER_ROTATION = 360;
  public static final int SECONDS_PER_MINUTE = 60;
  public static final double MAX_SPEED_METERS_PER_SECOND = 2.5;
  public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
  
  //Motor IDs
  // Drive IDs are in JSON files
  public static final int ARM_MOTOR = 9;
  public static final int INTAKE_MOTOR_ONE = 10;
  public static final int TOP_ROLLER_MOTOR = 11;
  public static final int INTAKE_MOTOR_TWO = 12;

  //Current Limits
  public static final int CURRENT_LIMIT_ARM = 30;
  public static final int CURRENT_LIMIT_INTAKE = 45;
  public static final int CURRENT_LIMIT_INTAKE_SPIKE = 80;
  public static final int CURRENT_LIMIT_TOP_ROLLER = 20;
  
  //Speeds and Positions
  public static final double INTAKE_MAX_SPEED = 7000;
  public static final double INTAKE_SPEED_CUBE = 350;
  public static final double OUTTAKE_SPEED_CUBE = -15;
  
  public static final double TOP_ROLLER_SPEED_CUBE = 1.0; //FIXME

  public static final double OUTTAKE_L2_SPEED = 300; //FIXME
  public static final double SHOOT_L2_SPEED = 0.2;
  public static final double SHOOT_L2_POSITION = -75; //FIXME
  public static final double SHOOT_L2_BACKWARDS_SPEED = 0.4;
  public static final double SHOOT_L2_BACKWARDS_POSITION = 0; //FIXME

  public static final double SHOOT_L3_CUBE_SPEED = 0.6;
  public static final double SHOOT_L3_BACKWARDS_SPEED = 1;
  public static final double SHOOT_L3_BACKWARDS_POSITION = 0;//FIXME
  public static final double SHOOT_L3_POSITION = -75;//FIXME
  
  public static final double OUTPUT_RANGE_MIN = -1;
  public static final double OUTPUT_RANGE_MAX = 1;
  
  public static final double DOWN_POSITION = -137; //FIXME
  public static final double DOWN_POSITION_BACKWARDS = 0;//FIXME
  public static final double REST_POSITION = 0;

  public static final double ARM_MAX_SPEED = 300;
  public static final double ARM_MAX_ACCELERATION = 300;
  public static final double ARM_POSITION_TOLERANCE = 1;

  //PIDs
  public static final int INTAKE_PID_SLOT_VELOCITY = 0;
  public static final double INTAKE_PID0_P = 1.0047e-17;
  public static final double INTAKE_PID0_I = 0;
  public static final double INTAKE_PID0_D = 0;
  public static final double INTAKE_PID0_F = 1.5e-5;
  public static final double INTAKE_PID0_IZ = 0;

  public static final double INTAKE_PID1_P = 0;
  public static final double INTAKE_PID1_I = 0;
  public static final double INTAKE_PID1_D = 0;
  public static final double INTAKE_PID1_F = 0.0;
  public static final double INTAKE_PID1_IZ = 0;

  public static final double INTAKE_KS = 0.36666;
  public static final double INTAKE_KV = 0.1621;
  public static final double INTAKE_KA = 0.0074968;

  public static final double INTAKE_VOLTAGE_HOLDING_CUBE = 0.75;

  public static final int ARM_PID_SLOT_VELOCITY = 0;
  public static final double ARM_PID0_P = 0.010086;
  public static final double ARM_PID0_I = 0;
  public static final double ARM_PID0_D = 0;
  public static final double ARM_PID0_F = 0.0;
  public static final double ARM_PID0_IZ = 0;

  public static final double ARM_KS = 0.38203;
  public static final double ARM_KV = 0.01656;
  public static final double ARM_KA = 0.0813398;

  public static final double ARM_PID_PROFILED_P = 0;
  public static final double ARM_PID_PROFILED_I = 0;
  public static final double ARM_PID_PROFILED_D = 0;

  public static final int ARM_GEAR_RATIO = 64; //FIXME

  public static final int ARM_PID_SLOT_POSITION = 0;
  
  public static final PIDFConfig xAutoPID = new PIDFConfig(5, 0, 0);
  public static final PIDFConfig yAutoPID = new PIDFConfig(5, 0, 0);
  public static final PIDFConfig angleAutoPID = new PIDFConfig(1, 0, 0.005);

  //Balance Auto Constants
  public static final double DRIVE_CLIMB_RISING_THRESHOLD = 16;
  public static final double DRIVE_CLIMB_LEVEL_THRESHOLD = 10;
  public static final double DRIVE_CLIMB_FAST_SPEED = 0.6; //FIXME Was 1.7
  public static final double DRIVE_CLIMB_FALLING_THRESHOLD = 15;
  public static final double DRIVE_CLIMB_RISING_SPEED = 1.1;
  public static final double DRIVE_CROSS_RISING_THRESHOLD = -10;
  public static final double DRIVE_CROSS_FALLING_THRESHOLD = -5;

}
