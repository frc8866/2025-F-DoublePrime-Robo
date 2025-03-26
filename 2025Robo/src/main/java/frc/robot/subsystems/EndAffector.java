package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndAffector extends SubsystemBase{
    public TalonFX algaemotor = new TalonFX(15);
    public TalonFX coralmotor = new TalonFX(16);
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private Slot0Configs slot0 = talonFXConfiguration.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    public EndAffector () {
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


    algaemotor.getConfigurator().apply(talonFXConfiguration); // aplies motion magic to the motor
    coralmotor.getConfigurator().apply(talonFXConfiguration);
    algaemotor.setNeutralMode(NeutralModeValue.Brake); // setting the motor to brake when not used by driver
    coralmotor.setNeutralMode(NeutralModeValue.Brake);
    

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Algea Motor Velocity", AlgeaVelocity());
        SmartDashboard.putNumber("Algea Motor Current Spike (StatorCurrent)", AlgeaCurrent()); // S
        SmartDashboard.putNumber("Coral Motor Current Spike (StatorCurrent)", CoralCurrent());
        SmartDashboard.putNumber("Coral Motor Velocity", CoralVelocity());
        SmartDashboard.putNumber("Algeamotor Setpoint", algaemotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Coralmotor Setpoint", coralmotor.getPosition().getValueAsDouble());

    }


    // Functions to help not repeating code

    
    public double AlgeaCurrent() { //Gets Motor's Current Output
        return algaemotor.getStatorCurrent().getValueAsDouble();
      }
    
    public double AlgeaVelocity() { //Gets Velocity of Motor
        return algaemotor.getVelocity().getValueAsDouble();
      }
    
    public double CoralCurrent() { //Gets Motor's Current Output
        return coralmotor.getStatorCurrent().getValueAsDouble();
      }
    
    public double CoralVelocity() { //Gets Velocity of Motor
        return coralmotor.getMotorVoltage().getValueAsDouble();
      }



    // Commands for algea

    public Command algeaintake(double speed_positive_value) {
        return new Command() {
            @Override
            public void initialize() {
              // Initialization code, such as resetting encoders or PID controllers
            }
      
            @Override
            public void execute() {
      
              algaemotor.set(speed_positive_value);
            }
      
            @Override
            public void end(boolean interrupted) {
              algaemotor.set(0);
            }
      
            @Override
            public boolean isFinished() {
              return AlgeaVelocity() > 40; // Check if the setpoint is reached; // Check if the setpoint is reached
            }
          };
    }
    public Command algeaouttake(double speed_positive_value) {
        return new Command() {
            @Override
            public void initialize() {
              // Initialization code, such as resetting encoders or PID controllers
              
            }
      
            @Override
            public void execute() {
      
              algaemotor.set(-speed_positive_value);
            }
      
            @Override
            public void end(boolean interrupted) {
              algaemotor.set(0);
            }
      
            @Override
            public boolean isFinished() {
              return algaemotor.getVelocity().getValueAsDouble() > 40; // Check if the setpoint is reached; // Check if the setpoint is reached
            }
          };
    }

    // Commands for coral

  public Command backdrive(double backdrive_speed) {
        return new Command() {
            @Override
            public void initialize() {
              // Initialization code, such as resetting encoders or PID controllers
            }
      
            @Override
            public void execute() {
      
              algaemotor.set(backdrive_speed);
            }
      
            @Override
            public void end(boolean interrupted) {
            }
      
            @Override
            public boolean isFinished() {
              return false;
            }
          };
      
    }

    public Command Coralcmd(double speed) {
      return new Command() {
        @Override
        public void initialize() {
          // Initialization code, such as resetting encoders or PID controllers
        }
  
        @Override
        public void execute() {
          algaemotor.set(speed);
        }
  
        @Override
        public void end(boolean interrupted) {
          algaemotor.set(0);
        }
  
        @Override
        public boolean isFinished() {
          return false;
        }
      };

    }



}
