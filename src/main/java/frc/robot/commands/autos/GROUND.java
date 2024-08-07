package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.LinearRegression;
import frc.FSLib.math.PID;
import frc.FSLib.math.simplePID;
import frc.FSLib.util.AngularVelocity;
import frc.robot.Constants.MapConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperStateMachine;
import frc.robot.Constants.UpperStateMachine.UpperState;
import frc.robot.subsystems.Upper;

public class GROUND extends Command {

  private Upper s_Upper;

  private final PID elbowPID = new PID(
    UpperConstants.elbowKP,
    UpperConstants.elbowKI,
    UpperConstants.elbowKD,
    UpperConstants.elbowiWindup,
    UpperConstants.elbowiLimit
  );

  private final simplePID shooterPID = new simplePID(
      UpperConstants.shooterKP,
      UpperConstants.shooterKD
  );

  private Timer timer = new Timer();

  public GROUND(Upper s_Upper) {
    this.s_Upper = s_Upper;
    addRequirements(s_Upper);
  }

  @Override
  public void initialize() {
    UpperConstants.INTAKE_GROUND_SPEED = AngularVelocity.fromRevPM(-3000);
  }

  @Override
  public void execute() {
    s_Upper.setLeftShooter(0);
    s_Upper.setRightShooter(0);
    if (!s_Upper.hasNote()) {
      RobotConstants.upperState = UpperState.GROUND;
      s_Upper.setElbow(-elbowPID.calculate(UpperStateMachine.elbowTarget.getRotations() - s_Upper.getElbowRotation()));
      s_Upper.setIntake(UpperStateMachine.intakeTarget.getRevPM() / UpperConstants.INTAKE_MAX_RPM);
    } else {
        RobotConstants.upperState = UpperState.DEFAULT;
        s_Upper.setIntake(UpperStateMachine.intakeTarget.getRevPM() / UpperConstants.INTAKE_MAX_RPM);
    }
  }

  @Override
  public void end(boolean interrupted){
    // System.out.println("GROUND end");
  }

  @Override
  public boolean isFinished() {
    if (s_Upper.hasNote()) {
      return true;
    } else {
      return false;
    }
  }
}