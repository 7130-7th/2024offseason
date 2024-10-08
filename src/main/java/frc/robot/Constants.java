package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.FSLib.limelight.LimelightHelpers;
import frc.FSLib.math.LinearRegression;
import frc.FSLib.math.PolynomialRegression;
import frc.FSLib.swerve.SwerveModuleConstants;
import frc.FSLib.util.AngularVelocity;
import frc.robot.Constants.UpperStateMachine.UpperState;
import frc.robot.subsystems.Upper;
import frc.robot.subsystems.Vision;

public class Constants {

    public static final class RobotConstants {
        public static final String canbusName = "GTX7130";
        public static final int DriverControllerID = 0;
        public static UpperState upperState = UpperState.DEFAULT;
        public static Rotation2d gyroYaw = new Rotation2d();
        public static SwerveModulePosition[] swervePositions = new SwerveModulePosition[4];
        public static Pose2d odometryPose = new Pose2d();
        private final Vision s_Vision = new Vision();
    }

    public static final class FieldConstants {
        public static final Translation2d redSpeakerCoord = new Translation2d(16.618, 5.442);
        public static final Translation2d blueSpeakerCoord = new Translation2d(0, 5.442);
    }

    public static final class UpperConstants {
    
        public static final int leftElbowMotorID = 17;
        public static final int rightElbowMotorID = 18;
        public static final int leftShooterMotorID = 41;
        public static final int rightShooterMotorID = 42;
        public static final int intakeMotorID = 43;

        public static final int leftLimitSwitchID = 7;
        public static final int rightLimitSwitchID = 8;

        public static final int elbowCancoderID = 4;
        public static final double elbowCancoderOffset = -0.00709;

        public static final int ledLength = 15;
        public static final int ledPwmPort = 7;

        public static boolean speedLimitEnabled = true;
    
        public static final double elbowKP = 6.5;
        public static final double elbowKI = 0.0;
        public static final double elbowKD = 0.5;
        public static final double elbowiWindup = 0.0;
        public static final double elbowiLimit = 0.0;
    
        public static final double shooterKP = 0.00001;
        public static final double shooterKD = 0.0;
    
        public static final Rotation2d ELBOW_AMP_POS = Rotation2d.fromRotations(0.013916);
        public static final Rotation2d ELBOW_DEFAULT_POS = Rotation2d.fromRotations(-0.203867);
        public static final Rotation2d ELBOW_ENDGAME_POS = Rotation2d.fromRotations(-0.248);
        public static final Rotation2d ELBOW_GROUND_POS = Rotation2d.fromRotations(-0.247);
        public static final Rotation2d ELBOW_PREENDGAME_POS = Rotation2d.fromRotations(0.01);

        public static final AngularVelocity INTAKE_AMP_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_DEFAULT_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_ENDGAME_SPEED = AngularVelocity.fromRevPM(0);
        public static AngularVelocity INTAKE_GROUND_SPEED = AngularVelocity.fromRevPM(-3000);
        public static final AngularVelocity INTAKE_PREENDGAME_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity INTAKE_SHOOT_SPEED = AngularVelocity.fromRevPM(-3000);
        public static final AngularVelocity INTAKE_SPEAKER_SPEED = AngularVelocity.fromRevPM(0);

        
        public static final double INTAKE_MAX_RPM = 5000;

        public static final AngularVelocity SHOOTER_AMP_SPEED = AngularVelocity.fromRevPM(-1000);
        public static final AngularVelocity SHOOTER_DEFAULT_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity SHOOTER_ENDGAME_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity SHOOTER_GROUND_SPEED = AngularVelocity.fromRevPM(0);
        public static final AngularVelocity SHOOTER_PREENDGAME_SPEED = AngularVelocity.fromRevPM(0);

        public static final double SHOOTER_LEGAL_RPM = 3500;

    }

    public static final class UpperStateMachine {

        public static enum UpperState {
            AMP,
            DEFAULT,
            ENDGAME,
            GROUND,
            PREENDGAME,
            SHOOT,
            SPEAKER,
            BASE
        }

        public static Rotation2d elbowTarget;
        public static AngularVelocity intakeTarget;
        public static AngularVelocity shooterTarget;

        public UpperStateMachine () {
            elbowTarget = UpperConstants.ELBOW_DEFAULT_POS;
            intakeTarget = UpperConstants.INTAKE_DEFAULT_SPEED;
            shooterTarget = UpperConstants.SHOOTER_DEFAULT_SPEED;
        }

        public void update () {
            switch (RobotConstants.upperState) {
                case AMP:
                    elbowTarget = UpperConstants.ELBOW_AMP_POS;
                    intakeTarget = UpperConstants.INTAKE_AMP_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_AMP_SPEED;
                    break;
                case DEFAULT:
                    elbowTarget = UpperConstants.ELBOW_DEFAULT_POS;
                    intakeTarget = UpperConstants.INTAKE_DEFAULT_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_DEFAULT_SPEED;
                    break;
                case ENDGAME:
                    elbowTarget = UpperConstants.ELBOW_ENDGAME_POS;
                    intakeTarget = UpperConstants.INTAKE_ENDGAME_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_ENDGAME_SPEED;
                    break;
                case GROUND:
                    elbowTarget = UpperConstants.ELBOW_GROUND_POS;
                    intakeTarget = UpperConstants.INTAKE_GROUND_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_GROUND_SPEED;
                    break;
                case PREENDGAME:
                    elbowTarget = UpperConstants.ELBOW_PREENDGAME_POS;
                    intakeTarget = UpperConstants.INTAKE_PREENDGAME_SPEED;
                    shooterTarget = UpperConstants.SHOOTER_PREENDGAME_SPEED;
                    break;
                case SHOOT:
                        intakeTarget = UpperConstants.INTAKE_SHOOT_SPEED;
                    break;
                case BASE:
                    elbowTarget = UpperConstants.ELBOW_DEFAULT_POS;
                    intakeTarget = UpperConstants.INTAKE_DEFAULT_SPEED;
                    shooterTarget = AngularVelocity.fromRevPM(-4000);
                case SPEAKER:
                    double[] tpcs = LimelightHelpers.getTargetPose_CameraSpace("limelight");
                    double tz = tpcs[2];
                    System.out.println(tz);
                    double distance = DriverStation.getAlliance().get() == Alliance.Red ? 
                        RobotConstants.odometryPose.getTranslation().getDistance(FieldConstants.redSpeakerCoord)
                        : RobotConstants.odometryPose.getTranslation().getDistance(FieldConstants.blueSpeakerCoord);
                        if (tz>=1) {
                    elbowTarget = Rotation2d.fromRotations(
                        LinearRegression.calculate(MapConstants.DISTANCE_TO_ELBOW_AND_SHOOTER, tz, 1)
                        // PolynomialRegression.predictDeg(tz)
                    );
                    if (RobotConstants.upperState == UpperState.SHOOT) {
                        intakeTarget = UpperConstants.INTAKE_SHOOT_SPEED;
                    } else {
                        intakeTarget = UpperConstants.INTAKE_SPEAKER_SPEED;
                    }
                    shooterTarget = AngularVelocity.fromRadPM(
                        // LinearRegression.calculate(MapConstants.DISTANCE_TO_ELBOW_AND_SHOOTER, distance, 2)
                        -3000
                    );
                    SmartDashboard.putNumber("distance to Speaker", distance);
                }
            }
        }
    }

    public static final class SwerveConstants {
        public static final double axisDeadBand = 0.05;
        public static final int pigeon1 = 1;
        public static final boolean invertGyro = false;

        /* Drivetrain Constants */
        public static final double trackWidth = 0.583; // meters, length between two side's wheels, need to adjust
        public static final double wheelBase = 0.583; // meters, length between same side's wheels, need to adjust
        public static final double driveBaseRaius = 0.35234;
        public static final double wheelDiameter = Units.inchesToMeters(4.0); // need to adjust
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        // public static final double openLoopRamp = 0.25;
        // public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (6.12244897959 / 1.0); // 6.12:1 (6.12244897959), for MK4i(L3)
        public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1, for MK4i(all)

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 10;
        public static final int driveContinuousCurrentLimit = 40;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.008;
        public static final double angleKI = 0.05;
        public static final double angleKD = 0.005;

        /* Angle Motor Auto-Facing PID Values */
        public static final double faceKP = 0.8;
        public static final double faceKI = 0.0;
        public static final double faceKD = 0.1;
        public static final double faceiWindup = 0.0;
        public static final double faceiLimit = 0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.08; // 0.12
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0025; 
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor = wheelCircumference / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxModuleSpeed = 4; // m/s
        public static final double maxModuleAccleration = 3; // m/s
        public static final double maxAngularVelocity = 13.5; // rad/s
        public static final double maxAngularAccleration = 12; // rad/s

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = false;
        public static final boolean angleInvert = false;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue canCoderInvert = SensorDirectionValue.CounterClockwise_Positive;

        /* Field Oriented */
        public static boolean fieldOriented = true;

        /* Slow Mode */
        public static boolean slow = false;

        public static final Translation2d LFModuleOffset = new Translation2d(0.3, 0.3);
        public static final Translation2d RFModuleOffset = new Translation2d(0.3, -0.3);
        public static final Translation2d LRModuleOffset = new Translation2d(-0.3, 0.3);
        public static final Translation2d RRModuleOffset = new Translation2d(-0.3, -0.3);
        
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
        public static final int driveMotorID = 1;
        public static final int angleMotorID = 2;
        public static final int canCoderID = 0;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.590682);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
        public static final int driveMotorID = 11;
        public static final int angleMotorID = 12;
        public static final int canCoderID = 1;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.364394);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }


    /* Rear Left Module - Module 2 */
    public static final class Mod2 {
        public static final int driveMotorID = 21;
        public static final int angleMotorID = 22;
        public static final int canCoderID = 2;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.413911+0.043457);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Rear Right Module - Module 3 */
    public static final class Mod3 {
        public static final int driveMotorID = 31;
        public static final int angleMotorID = 32;
        public static final int canCoderID = 3;
        public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.06372);
        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(0.5, 0, 0.001), // Translation constants 
            new PIDConstants(0.08, 0, 0), // Rotation constants 
            maxModuleSpeed, 
            LFModuleOffset.getNorm(), // Drive base radius (distance from center to furthest module) 
            new ReplanningConfig(true, true)
        );
    }

    // public static final class PoseEstimator {

    //     private static SwerveDrivePoseEstimator m_Estimator;

    //     public PoseEstimator () {
    //         m_Estimator = new SwerveDrivePoseEstimator(
    //             new SwerveDriveKinematics(
    //                 SwerveConstants.LFModuleOffset, 
    //                 SwerveConstants.RFModuleOffset, 
    //                 SwerveConstants.LRModuleOffset, 
    //                 SwerveConstants.RRModuleOffset
    //             ),
    //             RobotConstants.gyroYaw,
    //             RobotConstants.swervePositions,
    //             RobotConstants.odometryPose
    //         );
    //     }

    //     public void update () {
    //             m_Estimator.update(RobotConstants.gyroYaw, RobotConstants.swervePositions);
    //             boolean doVisionUpdate = true;
    //             LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    //             if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
    //                 // 這個很重要但有bug
    //                 // if(mt1.rawFiducials[0].ambiguity > 0.7) doVisionUpdate = false;
    //                 // if(mt1.rawFiducials[0].distToCamera > 3) doVisionUpdate = false;
    //             }
    //             if (mt1.tagCount == 0) doVisionUpdate = false;
    //             if (doVisionUpdate) {
    //                 m_Estimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
    //                 m_Estimator.addVisionMeasurement(
    //                     mt1.pose,
    //                     mt1.timestampSeconds
    //                 );
    //             }
    //         }

    //     public Pose2d getEstimatedPosition () {
    //         return m_Estimator.getEstimatedPosition();
    //     }
    // }

    public static final class MapConstants {
        
        public static final double[][] SHOOTER_RPM_TO_OUTPUT = {
            {-Double.MAX_VALUE, -1},
            {-5000, -1},
            {0.0, 0.0},
            {5000, 1},
            {Double.MAX_VALUE, 1}
        };

        public static final double[][] DISTANCE_TO_ELBOW_AND_SHOOTER = {
            {-Double.MAX_VALUE, -0.243, -4000},
            {1.0, -0.205, -4000},
            {2.5, -0.185, -3500},
            {4.0, -0.145, -4000},
            {Double.MAX_VALUE, -0.08, -4000}
        };

    }

    public static final class LimeLight {
        public static final double KPDefault = 0.0033; // 0.013
        public static final double KIDefault = 0.001;
        public static final double KDDefault = 0.00001;
        public static final double WindupDefault = 0;
        public static final double LimitDefault = 0;
        public static final double SmoothDefault = 0.7;
    
        public static final String KPKey = "kp";
        public static final String KIKey = "ki";
        public static final String KDKey = "kd";
        public static final String WindupKey = "wind";
        public static final String LimKey = "lim";
        public static final String SmoothKey = "smooth"; 
      }
}
