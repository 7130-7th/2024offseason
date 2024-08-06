// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopUpper;
import frc.robot.commands.autos.DEFAULT;
import frc.robot.commands.autos.GROUND;
import frc.robot.commands.autos.SHOOT;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Upper;

public class RobotContainer {

  private final XboxController controller = new XboxController(RobotConstants.DriverControllerID);

  private static Swerve s_Swerve = new Swerve();
  private static Upper s_Upper = new Upper();
  // private static Vision s_Vision = new Vision();

  private final TeleopSwerve teleopSwerve = new TeleopSwerve(s_Swerve, controller);
  private final TeleopUpper teleopUpper = new TeleopUpper(s_Upper, controller);

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public RobotContainer() {
    s_Swerve.setYaw(Rotation2d.fromDegrees(0));
    s_Swerve.setPose(new Pose2d(new Translation2d(13.56, 5.56), Rotation2d.fromDegrees(180)));
    s_Swerve.setDefaultCommand(teleopSwerve);
    s_Upper.setDefaultCommand(teleopUpper);
    
    // s_Upper.setDefaultCommand(new RunCommand(()->{
      
    //   if (controller.getYButton()) s_Upper.setElbow(0.2);
    //   else if (controller.getAButton()) s_Upper.setElbow(-0.2);
    //   else s_Upper.setElbow(0);

    //   if (controller.getRightBumper()) s_Upper.setIntake(-0.5);
    //   else s_Upper.setIntake(0);

    //   if (controller.getRightTriggerAxis() > 0.05) {
    //     s_Upper.setRightShooter(-0.8);
    //     s_Upper.setIntake(-1);
    //   }
    // }, s_Upper));

    // NamedCommands.registerCommand("AutoAim", new AutoAim(s_Upper));
    // NamedCommands.registerCommand("AutoIntake", new AutoIntake(s_Upper));
    // NamedCommands.registerCommand("AutoShoot", new AutoShoot(s_Upper));

    SmartDashboard.putData("auto chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup (
      new SHOOT(s_Upper),
      new ParallelCommandGroup(
        new GROUND(s_Upper),
        new PathPlannerAuto("MB-X2")
      ),
      // new DEFAULT(s_Upper),
      new SHOOT(s_Upper)
    //   new ParallelCommandGroup(
    //     new PathPlannerAuto("MB-X1"),
    //     new GROUND(s_Upper)
    //     ),
    //   new SHOOT(s_Upper),
    //   new ParallelCommandGroup(
    //     new PathPlannerAuto("X1-X2"),
    //     new GROUND(s_Upper)
    //   ),
    //   new SHOOT(s_Upper),
    //   new ParallelCommandGroup(
    //     new PathPlannerAuto("X2-X3"),
    //     new GROUND(s_Upper)
    //   ),
    //   new SHOOT(s_Upper)
    );
  }
}
