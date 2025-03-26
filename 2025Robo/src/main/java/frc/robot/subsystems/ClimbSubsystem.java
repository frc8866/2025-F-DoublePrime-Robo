package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.*;
import frc.robot.Constants;
import java.util.List;

public class ClimbSubsystem extends SubsystemBase{
    public TalonFX climbmotor = new TalonFX(20);
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private Slot0Configs slot0 = talonFXConfiguration.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

// PID Controller that can be used instead of Motion Magic
    //public PIDController ClimbUpPID = new PIDController(0.07, 0, 0);
    //public PIDController ClimbDownPID = new PIDController(0.03, 0, 0);


    public ClimbSubsystem () {
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


    climbmotor.getConfigurator().apply(talonFXConfiguration); // aplies motion magic to the motor
    climbmotor.setNeutralMode(NeutralModeValue.Brake); // setting the motor to brake when not used by driver
    

    }


    @Override
    public void periodic() {

        SmartDashboard.putNumber("Climbmotor Setpoint", climbmotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Climbmotor Voltage", climbmotor.getMotorVoltage().getValueAsDouble());

    }


    public Command climbCommand(double position) {
        return new Command() {
            @Override
            public void initialize() {
              // Initialization code, such as resetting encoders or PID controllers
              
              
              
              //PID Way of doing it
              //ClimbDownPID.setSetpoint(position);
            }
      
            @Override
            public void execute() {
            
            
            // PID way of doing it
              // double speed = ClimbDownPID.calculate(climbmotor.getPosition().getValueAsDouble());
              // climbmotor.set(speed);


            // Motion Magic way of doing it
            climbmotor.setControl(m_request.withPosition(position).withFeedForward(0.15)); 
            //the feedforward is not required it just helps it being more personalized, like multiplying it by a constant

            }
      
            @Override
            public void end(boolean interrupted) {
              // No point really of having this because once motion magic finishes its command, it should not keep running
              // but I have it here just in case
              // Also for future reference, use VoltageOut to be 0, more accurate than speed = 0 because cutting the voltage out will be more accurate
              climbmotor.setControl(new VoltageOut(0));

            }
      
            @Override
            public boolean isFinished() {
              return false;
            }
          };
    }
}
