// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

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
    public static final int kDriverControllerPort = 0;

  }


  
  private static RobotState currentRobotState = RobotState.IDLE;
  private static Elevatorposition curentElevatorposition = Elevatorposition.L0;

  public static final double pivotl0 = 0;
  public static final double pivotl1 = -7.13134765625;
  public static final double pivotl2= -26.300390625;
  public static final double pivotl3 = -26.0193359375;
  public static final double pivotl4 = -23.8123046875;
  public static final double elevatorl0 = 0;
  public static final double elevatorl1 =;
  public static final double elevatorl2= ;
  public static final double elevatorl3 =;
  public static final double elevatorl4 = ;
  public static final double elevatoralgea =;
  public static final double backdrive_speed = 0.07;




public static enum RobotState {
  IDLE, // Robot is not doing anything
  MOVING, // Robot is driving
  CORAL_NONE, // Robot is picking up a game piece
  CORAL_HOLDING,
  SHOOTING, // Robot is shooting a game piece
  CLIMBING, // Robot is climbing
  ALGEA_NONE,
  ALGEA_HOLDING; // Robot has Algea intaked

}

public enum Elevatorposition {
  L4,
  L3,
  L2,
  L1,
  L0
}



//function for updating/setting a robot state
public static void setRobotState(RobotState newState) {
  currentRobotState = newState;
  System.out.println("Robot state updated to: " + newState);
}
public static RobotState getRobotState() {
  return currentRobotState;
}

public static Elevatorposition getElevatorState() {
  return curentElevatorposition;
}

public static void setElevatorState(Elevatorposition newState) {
  curentElevatorposition = newState;
  System.out.println("Robot state updated to: " + newState);
}

}
