package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.*;
import frc.robot.Constants;
import frc.robot.Constants.SetpointGroup;

public class ElevatorSubsystem  extends SubsystemBase {
    public TalonFX r_elevatormotor = new TalonFX(13);
    public TalonFX l_elevatormotor = new TalonFX(14);
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private Slot0Configs slot0 = talonFXConfiguration.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    public SetpointGroup activeSetpointGroup = Constants.SetpointGroup.AlgeaSetpoints;
    public PIDController elevatorPID = new PIDController(0.06, 0, 0);
    private double pivotStateSetpoint;

    public ElevatorSubsystem () {
    // motion magic stuff, comments are there for understanding
    slot0.kS = 0.25;
    slot0.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 0.3; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 400;// Target jerk of 1600 rps/s/s (0.1 seconds)


    r_elevatormotor.getConfigurator().apply(talonFXConfiguration); // aplies motion magic to the motor
    l_elevatormotor.getConfigurator().apply(talonFXConfiguration);
    r_elevatormotor.setNeutralMode(NeutralModeValue.Brake); // setting the motor to brake when not used by driver
    l_elevatormotor.setNeutralMode(NeutralModeValue.Brake);
    

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Right Elevator Motor Velocity", r_elevatormotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Left Elevator Motor Velocity", l_elevatormotor.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Right Elevator Motor Current Spike (StatorCurrent)", r_elevatormotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Left Elevator Motor Current Spike (StatorCurrent)", l_elevatormotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Elevator Setpoint", rightelevator_position());
        SmartDashboard.putNumber("Left Elevator Setpoint", l_elevatormotor.getPosition().getValueAsDouble());
        
        //setting elevator left to be a follower so I do not have to code it twice, just for right and it will apply it to left
        l_elevatormotor.setControl(new Follower(r_elevatormotor.getDeviceID(), true));
    }



    //functions


    // gets value of right_elevator
    public double rightelevator_position() {
        return r_elevatormotor.getPosition().getValueAsDouble();
    }


    // can be used as a Tolerance function, where it can check if it is inside a tolerance and then return a true or false used for the 'isFinished' of a command rather than doing it inside the 'isfinished' function.
      public Boolean check(double position, double tolerance_range) {
        double currentposition = rightelevator_position();
        if (currentposition - tolerance_range < position && currentposition + tolerance_range > position) {
          return true;
    
        } else {
          return false;
        }
      }

    public Command elevatorCommandMotionMagic(int value) {
        return new Command() {
          // Define a tolerance (adjust as needed based on your sensor units)
          private final double kTolerance = 0.2;

          
          @Override
          public void initialize() {
            // Optionally reset any state or encoders if needed
    
          }
    
          @Override
          public void execute() {
    
            // Command the leader motor using Motion Magic with feedforward.
            // (Since re is meant to follow le, remove direct control of re here.)
    
            r_elevatormotor.setControl(m_request.withPosition(activeSetpointGroup.getSetpoints().get(value)).withFeedForward(0.4).withEnableFOC(true));
          }

          @Override
          public void end(boolean interrupted) {
            // Once finished (or interrupted), stop the motors.
            r_elevatormotor.setControl(new VoltageOut(0));
            // If you’re using a follower, you can let it follow automatically.
            l_elevatormotor.setControl(new VoltageOut(0)); //this line is not useful

          }

          @Override
          public boolean isFinished() {
            //METHOD 1
            // End the command once the error is within tolerance.
            //double error = Math.abs(r_elevatormotor.getPosition().getValueAsDouble() - activeSetpointGroup.getSetpoints().get(value));
            //return kTolerance > error; // if true, then ends command

            // METHOD 2 (has a range, rather than just a one sided tolerance) 
            //BETTER THAN METHOD 1
            // another way of doing it using the function 'check'
            double position = r_elevatormotor.getPosition().getValueAsDouble();
            return check(position, kTolerance);
          }
        };
      }
      public Command elevatorCommandPID(double position) {
        return new Command() {
            @Override
            public void initialize() {
              // Initialization code, such as resetting encoders or PID controllers
              
              
              
              //PID Way of doing it
              elevatorPID.setSetpoint(position);
            }
      
            @Override
            public void execute() {
            
            
            // PID way of doing it
            double speed = elevatorPID.calculate(r_elevatormotor.getPosition().getValueAsDouble());
            r_elevatormotor.set(speed);


            // Motion Magic way of doing it
            //r_elevatormotor.setControl(m_request.withPosition(position).withFeedForward(0.15));
            //the feedforward is not required it just helps it being more personalized, like multiplying it by a constant

            }
      
            @Override
            public void end(boolean interrupted) {
              r_elevatormotor.set(0);
              r_elevatormotor.setControl(new VoltageOut(0));
              l_elevatormotor.setControl(new VoltageOut(0));

            }
      
            @Override
            public boolean isFinished() {
              return false;
            }
          };
    }



      public void SettingRobotState() {
        if (activeSetpointGroup == SetpointGroup.CoralSetpoints) {
            activeSetpointGroup = SetpointGroup.AlgeaSetpoints;
            Constants.setRobotState(Constants.RobotState.ALGEA);
        } else {
            activeSetpointGroup = SetpointGroup.CoralSetpoints;
            Constants.setRobotState(Constants.RobotState.IDLE);
        }
      }

      public Command elevatorStateCommand(int targetPosition, boolean is_pivot_up) {
        return new Command() {
          // Define a tolerance (adjust as needed based on your sensor units)

          
          @Override
          public void initialize() {
            // Optionally reset any state or encoders if needed
            if (targetPosition == 1) {
              Constants.setElevatorState(Constants.Elevatorposition.L1);
            } else if (targetPosition == 2) {
              Constants.setElevatorState(Constants.Elevatorposition.L2);
            } else if (targetPosition == 3) {
              Constants.setElevatorState(Constants.Elevatorposition.L3);
            } else if (targetPosition == 4) {
              Constants.setElevatorState(Constants.Elevatorposition.L4);
            } else {
              Constants.setElevatorState(Constants.Elevatorposition.L0);
            }

          }
    
          @Override
          public void execute() {
    
            // Command the leader motor using Motion Magic with feedforward.
            // (Since re is meant to follow le, remove direct control of re here.)
            if (is_pivot_up) {
              // Set flipsetpoint based on the desired elevator state.
              if (Constants.getElevatorState() == Constants.Elevatorposition.L1) {
                pivotStateSetpoint = Constants.pivotl1;
        
              } else if (Constants.getElevatorState() == Constants.Elevatorposition.L2) {
                pivotStateSetpoint = Constants.pivotl2;
              } else if (Constants.getElevatorState() == Constants.Elevatorposition.L3) {
                pivotStateSetpoint = Constants.pivotl3;
              } else if (Constants.getElevatorState() == Constants.Elevatorposition.L4) {
                pivotStateSetpoint = Constants.pivotl4;
        
                // BargeShoot
        
              }
              
              elevatorPID.setSetpoint(pivotStateSetpoint);
          }}

          @Override
          public void end(boolean interrupted) {

          }

          @Override
          public boolean isFinished() {
              return false;
          }
        };
      }

  
}