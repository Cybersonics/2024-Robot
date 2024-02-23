package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {

  private static Pneumatics instance;

  private DoubleSolenoid _launcher = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PneumaticConstants.launcherUp, Constants.PneumaticConstants.launcherDown);

  Compressor pcmCompressor = new Compressor(Constants.PneumaticConstants.compressorInput, PneumaticsModuleType.REVPH);

  /** Creates a new Pnuematics. */
  private Pneumatics() {
    pcmCompressor.enableAnalog(70,120);
    //pcmCompressor.enableDigital();
    //pcmCompressor.disable();

    _launcher.set(Value.kReverse);
  }

  public static Pneumatics getInstance() {
    if (instance == null) {
      instance = new Pneumatics();
    }
    return instance;
  }

  public void launcherToggle() {
    _launcher.toggle();
  }

  public void launcherUp() {
    _launcher.set(Value.kForward);
  }

  public void launcherDown() {
    _launcher.set(Value.kReverse);
  }
}
