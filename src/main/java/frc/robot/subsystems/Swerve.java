// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.RobotConstants;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro = new Pigeon2(SwerveConstants.pigeon1, RobotConstants.canbusName);

  public SwerveModule[] mSwerveMods = new SwerveModule[] {
    new SwerveModule(0, SwerveConstants.Mod0.constants),
    new SwerveModule(1, SwerveConstants.Mod1.constants),
    new SwerveModule(2, SwerveConstants.Mod2.constants),
    new SwerveModule(3, SwerveConstants.Mod3.constants)
  };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    SwerveConstants.LFModuleOffset, 
    SwerveConstants.RFModuleOffset, 
    SwerveConstants.LRModuleOffset, 
    SwerveConstants.RRModuleOffset
  );;

  private SwerveDriveOdometry swerveOdometry = new SwerveDriveOdometry(kinematics, getYaw(), getPositions());;

  private Field2d m_field = new Field2d();

  private double AngularVel;

  public Swerve() {
    gyro.setYaw(0);

    AutoBuilder.configureHolonomic(
      this::getOdometryPose, 
      this::setPose, 
      this::getSpeeds, 
      this::driveRobotRelative, 
      SwerveConstants.pathFollowerConfig,
      () -> {
          if (DriverStation.getAlliance().isPresent()) return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
          return false;
      },
      this
    );

    // Set up custom logging to add the current path to a m_field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> m_field.getObject("path").setPoses(poses));
    SmartDashboard.putData("Field", m_field);
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.maxModuleSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxModuleSpeed);
    for (SwerveModule mod : mSwerveMods) { 
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getOdometryPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getOdometryPose().getRotation()));
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  public ChassisSpeeds getSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    return positions;
  }

  public Rotation2d getYaw(){
    return Rotation2d.fromDegrees(gyro.getYaw().getValue());
  }

  public void setYaw(Rotation2d pos) {
    gyro.setYaw(pos.getDegrees());
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    AngularVel = Math.max(gyro.getAngularVelocityZWorld().getValueAsDouble(), AngularVel);
    m_field.setRobotPose(swerveOdometry.getPoseMeters());
    RobotConstants.odometryPose = getOdometryPose();
    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putNumber("Yaw", getYaw().getDegrees());
    SmartDashboard.putNumber("AngularVel", AngularVel);
    SmartDashboard.putNumber("OdometryX", getOdometryPose().getX());
    SmartDashboard.putNumber("OdometryY", getOdometryPose().getY());
    SmartDashboard.putNumber("OdometryRotation", getOdometryPose().getRotation().getRotations());
  }
}
