package frc.robot.commands.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.PID;
import frc.robot.Constants;
import frc.robot.Constants.LimeLight;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class AIM extends Command {
  private Swerve s_Swerve;
  private Vision s_Vision;
  private XboxController driver;

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;
  private double ema;

  public double KP = Constants.LimeLight.KPDefault;
  public double KI = Constants.LimeLight.KIDefault;
  public double KD = Constants.LimeLight.KDDefault;
  public double WindUp = Constants.LimeLight.WindupDefault;
  public double Limit = Constants.LimeLight.LimitDefault;
  public double Smooth = Constants.LimeLight.SmoothDefault;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry tpcs = table.getEntry("targetpose_cameraspace");
  
  private PID facingPID = new PID(KP, KI, KD, WindUp, Limit);
  

  public AIM(Swerve s_Swerve, Vision s_Vision, XboxController controller) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    this.driver = controller;
    addRequirements(s_Swerve);
    addRequirements(s_Vision);
  }

  @Override
  public void execute() {
    Preferences.initDouble(LimeLight.KPKey, KP);
    Preferences.initDouble(LimeLight.KIKey, KI);
    Preferences.initDouble(LimeLight.KDKey, KD);
    Preferences.initDouble(LimeLight.WindupKey, WindUp);
    Preferences.initDouble(LimeLight.LimKey, Limit);
    Preferences.initDouble(LimeLight.SmoothKey, Smooth);

    KP = Preferences.getDouble(LimeLight.KPKey, KP);
    KI = Preferences.getDouble(LimeLight.KIKey, KI);
    KD = Preferences.getDouble(LimeLight.KDKey, KD);
    WindUp = Preferences.getDouble(LimeLight.WindupKey, WindUp);
    Limit = Preferences.getDouble(LimeLight.LimKey, Limit);
    Smooth = Preferences.getDouble(LimeLight.SmoothKey, Smooth);

    // SmartDashboard.putNumber("tx", tx.getDouble(0.0));
    // SmartDashboard.putNumber("ty", ty.getDouble(0.0));
    // SmartDashboard.putNumber("tz", tpcs.getDoubleArray(new double[6])[2]);
    // SmartDashboard.putNumber("speed", rotationVal);
    
    // if (Math.abs(tx.getDouble(0.0))<=1) {
    //   ema = Smooth*tx.getDouble(0.0)+(1-Smooth)*ema;
    //   rotationVal = ema*1;
    //   // rotationVal=0;
    // }else{
    //   rotationVal = facingPID.calculate(tx.getDouble(0.0))*1;
    //   // rotationVal = MathUtil.applyDeadband(tx.getDouble(0), 0.05);
    // }

    // rotationVal = facingPID.calculate(tx.getDouble(0.0));

    ema = Smooth*facingPID.calculate(tx.getDouble(0.0))+(1-Smooth)*ema;
    rotationVal = ema;

    /* Drive */
    s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxModuleSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity, true,
        true);
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("bro ur teleop swerve is fucked");
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}