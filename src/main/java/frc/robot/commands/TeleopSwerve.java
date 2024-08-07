package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.PID;
import frc.FSLib.math.simplePID;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

public class TeleopSwerve extends Command {
  private Swerve s_Swerve;
  private Vision s_Vision;
  private XboxController driver;

  private static enum swerveState{
    DEFAULT,
    aimBot
  }

  private swerveState state = swerveState.DEFAULT;

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

  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  private double translationVal;
  private double strafeVal;
  private double rotationVal;

  public TeleopSwerve(Swerve s_Swerve,Vision s_Vision, XboxController controller) {
    this.s_Swerve = s_Swerve;
    this.s_Vision = s_Vision;
    this.driver = controller;
    addRequirements(s_Swerve);
  }

  @Override
  public void execute() {

    if (driver.getBackButton()) {
      s_Swerve.setYaw(Rotation2d.fromDegrees(0));
      s_Swerve.setPose(new Pose2d());
    }

    if (driver.getBButtonPressed()) {
      state = state == swerveState.DEFAULT? swerveState.aimBot : swerveState.DEFAULT;
    }

    /* Get Values, Deadband */
    translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(-driver.getLeftY()*0.6, Constants.SwerveConstants.axisDeadBand));
    strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(-driver.getLeftX()*0.6, Constants.SwerveConstants.axisDeadBand));

    switch (state) {
      case DEFAULT:
        rotationVal = 
      rotationLimiter
        .calculate(MathUtil.applyDeadband(-driver.getRightX()*0.6, Constants.SwerveConstants.axisDeadBand));
        break;
        
      case aimBot:
      ema = 0.8*facingPID.calculate(tx.getDouble(0.0))+(1-0.8)*ema;
      rotationVal = -ema;
    //   if (Math.abs(tx.getDouble(0.0))<=1) {
    //   ema = Smooth*tx.getDouble(0.0)+(1-Smooth)*ema;
    //   rotationVal = -ema*1;
    //   // rotationVal=0;
    // }else{
    //   rotationVal = -facingPID.calculate(tx.getDouble(0.0))*1;
    // }
        break;
    }
    // rotationVal = 
    //   rotationLimiter
    //     .calculate(MathUtil.applyDeadband(-driver.getRightX()*0.6, Constants.SwerveConstants.axisDeadBand));

    // if (driver.getLeftBumper()) {

    //   double zTarget;
    //   if (DriverStation.getAlliance().isPresent() ) {
    //     if (DriverStation.getAlliance().get() == Alliance.Blue) {
    //       zTarget = FieldConstants.blueSpeakerCoord.minus(s_Swerve.getOdometryPose().getTranslation()).getAngle().getRotations();
    //     } else {
    //       zTarget = FieldConstants.redSpeakerCoord.minus(s_Swerve.getOdometryPose().getTranslation()).getAngle().getRotations();
    //     }
    //   } else {
    //     zTarget = s_Swerve.getYaw().getRotations();
    //   }

    //   double z = s_Swerve.getYaw().getRotations();
    //   z -= Math.floor(z);
    //   if (z > 0.5) z -= 1;
    //   s_Swerve.drive(
    //     new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxModuleSpeed),
    //     zPID.calculate(z, zTarget) * SwerveConstants.maxAngularVelocity,
    //     true,
    //     false
    //   );

    // } else {
      s_Swerve.drive(
        new Translation2d(translationVal, strafeVal).times(SwerveConstants.maxModuleSpeed),
        rotationVal * Constants.SwerveConstants.maxAngularVelocity, true,
        true
      );
    // }
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished(){
    return false;
  }
}