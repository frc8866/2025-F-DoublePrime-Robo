package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase{
    private final CANdle candle = new CANdle(0);
    private final Timer blinkTimer = new Timer();
    private final Animation rgbFadeAnimation = new RgbFadeAnimation(1.5, 1.5, 360);
    private final Animation rainbowAnimation = new RainbowAnimation(1.5, 1, 360, false, 8);
    private final Animation fireAnimation = new FireAnimation(1.5, 1.5, 360, 0.8, 0.2, false, 8);

  public static class Color {
    public final int R, G, B;

    public Color(int r, int g, int b) {
      R = r;
      G = g;
      B = b;
    }
  }


  // Predefined colors for flash mode.
  // When transitioning, the LED will alternate between off and a flash color.
  public static final class Colors {
    public static final Color off = new Color(0, 0, 0);
    // Flash color for ALGEA mode (matching a fire-like hue)
    public static final Color fireFlash = new Color(255, 64, 0);
    // Flash color for IDLE mode (bright white)
    public static final Color rainbowFlash = new Color(255, 255, 255);
  }

  public LEDSubsystem() {

    // Use device 0 for LEFT and device 1 for RIGHT.
    applyConfigs();
    blinkTimer.start();

    // Set initial animation based on the current state.
    var initialState = Constants.getRobotState();
    if (!DriverStation.isEnabled()) {
      animate(rgbFadeAnimation);
    } else {
    //  if (initialState == RobotState.ALGEA) {
      if (initialState == Constants.RobotState.ALGEA) {
        animate(rgbFadeAnimation);
      } else if (initialState == Constants.RobotState.IDLE) {
        animate(rainbowAnimation);
      } else {
        animate(rainbowAnimation);
      }
    }
  }

  private void applyConfigs() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.brightnessScalar = 1.0;
    config.stripType = LEDStripType.GRB;
    config.v5Enabled = true;
    config.disableWhenLOS = false; // Adjust if needed.
    candle.configAllSettings(config);
  }

  public void animate(Animation animation) {
    candle.animate(animation);
  }

  public void clearAnimation() {
    candle.clearAnimation(0);
  }

  // Sets the LED to a specific solid color (used during flash mode).
  public void setColor(Color color) {
    candle.setLEDs(color.R, color.G, color.B);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putString("LED/RobotState", Constants.getRobotState().toString());

    // If the robot is disabled, always run the RGBFade animation.
    if (!DriverStation.isEnabled()) {
      animate(fireAnimation);
    } else {
      var currentState = Constants.getRobotState();

      if (currentState == Constants.RobotState.ALGEA) {
        animate(rainbowAnimation);
      }
      if (currentState == Constants.RobotState.IDLE) {
        animate(fireAnimation);
        // Coral
      }
    } 
}
}
