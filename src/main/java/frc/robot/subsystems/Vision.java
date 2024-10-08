package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.FSLib.math.NoiseFilter;

public class Vision extends SubsystemBase {

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    private final NetworkTableEntry tx = table.getEntry("tx");
    private final NetworkTableEntry tpcs = table.getEntry("targetpose_cameraspace");
    private final NetworkTableEntry botpose = table.getEntry("botpose_wpiblue");

    private final NoiseFilter xFilter = new NoiseFilter(20);
    private final NoiseFilter yFilter = new NoiseFilter(20);
    private final NoiseFilter rFilter = new NoiseFilter(20);

    private Pose2d robotPose = new Pose2d();



    public double getTx () {
        return tx.getDouble(0);
    }

    public double getRobotX () {
        return botpose.getDoubleArray(new Double[6])[0];
    }

    public double getRobotY () {
        return botpose.getDoubleArray(new Double[6])[1];
    }

    public double getRobotRotation () {
        return botpose.getDoubleArray(new Double[6])[5];
    }

    public Pose2d getRobotPose () {
        return robotPose;
    }

    public void setpriorityId(){
        if (DriverStation.getAlliance().get()==DriverStation.Alliance.Blue) {
            table.getEntry("priorityid").setInteger(7);
        }else{
            table.getEntry("priorityid").setInteger(4);
        }
    }

    public boolean hasTarget () {
        return table.getEntry("tid").getBoolean(false);
    }

    public double getTz() {
        return tpcs.getDoubleArray(new double[6])[2];
    }
    
    @Override
    public void periodic() {
        if (hasTarget()) robotPose = new Pose2d(
            xFilter.getAverage(getRobotX()),
            yFilter.getAverage(getRobotY()),
            Rotation2d.fromDegrees(rFilter.getAverage(getRobotRotation()))
        );
        SmartDashboard.putNumberArray("robotPose", new double[]{robotPose.getX(), robotPose.getY(), robotPose.getRotation().getDegrees()});
        
    SmartDashboard.putNumber("tx", tx.getDouble(0.0));
    // SmartDashboard.putNumber("ty", ty.getDouble(0.0));
    SmartDashboard.putNumber("tz", tpcs.getDoubleArray(new double[6])[2]);
    // SmartDashboard.putNumber("speed", rotationVal);
    
    }
}
