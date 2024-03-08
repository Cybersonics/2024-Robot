package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pneumatics extends SubsystemBase {

  private static Pneumatics instance;

  private DoubleSolenoid _launcher = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PneumaticConstants.launcherUp, Constants.PneumaticConstants.launcherDown);
      
  private DoubleSolenoid _ampArm = new DoubleSolenoid(PneumaticsModuleType.REVPH,
      Constants.PneumaticConstants.AmpArmOut, Constants.PneumaticConstants.AmptArmIn);

  Compressor pcmCompressor = new Compressor(Constants.PneumaticConstants.compressorInput, PneumaticsModuleType.REVPH);

  /** Creates a new Pnuematics. */
  private Pneumatics() {
    pcmCompressor.enableAnalog(50,60);
    //pcmCompressor.enableDigital();
    //pcmCompressor.disable();

    // _launcher.set(Value.kReverse);    
    _ampArm.set(Value.kReverse);
    
    SmartDashboard.putBoolean("LauncherUp", IsLauncherUp());
  }

  public static Pneumatics getInstance() {
    if (instance == null) {
      instance = new Pneumatics();
    }
    return instance;
  }

  public void launcherToggle() {
    _launcher.toggle();
    SmartDashboard.putBoolean("LauncherUp", !IsLauncherUp());
  }

  public boolean IsLauncherUp() {
    return _launcher.get().equals(Value.kForward);
  }

  public void launcherUp() {
    _launcher.set(Value.kForward);
  }

  public void launcherDown() {
    _launcher.set(Value.kReverse);
  }

  public void ampArmToggle() {
    _ampArm.toggle();
  }

  public void ampArmOut() {
    _ampArm.set(Value.kForward);
  }

  public void ampArmIn() {
    _ampArm.set(Value.kReverse);
  }

  public boolean IsArmOut() {
    return _ampArm.get().equals(Value.kForward);
  }
}
