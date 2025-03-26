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

  private static final double pivotl1 = -7.13134765625;
  private static final double pivotl2= -26.300390625;
  private static final double pivotl3 = -26.0193359375;
  private static final double pivotl4 = -23.8123046875;

  public enum SetpointGroup {
    SETPOINTS1(List.of(0.0, 0.0, 0.0, 9.44423828125, 26.241796875, 5.0)),
    SETPOINTS2(List.of(0.0, 7.75048828125, 7.75048828125, 35.0, 4.5, 5.5));

    private final List<Double> setpoints;

    SetpointGroup(List<Double> setpoints) {
        this.setpoints = setpoints;
    }

    public List<Double> getSetpoints() {
        return setpoints;
    }
}


//function for updating/setting a robot state
public static void setRobotState(RobotState newState) {
  currentRobotState = newState;
  System.out.println("Robot state updated to: " + newState);
}

public static enum RobotState {
  IDLE, // Robot is not doing anything
  MOVING, // Robot is driving
  INTAKING, // Robot is picking up a game piece
  SHOOTING, // Robot is shooting a game piece
  CLIMBING, // Robot is climbing
  ALGEA; // Robot has Algea intaked
}

}
