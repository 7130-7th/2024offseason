package frc.robot.commands.autos;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.LinearRegression;
import frc.FSLib.math.PID;
import frc.FSLib.math.simplePID;
import frc.robot.Constants.MapConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperStateMachine;
import frc.robot.Constants.UpperStateMachine.UpperState;
import frc.robot.subsystems.Upper;

public class DEFAULT extends Command {

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

  public DEFAULT(Upper s_Upper) {
    this.s_Upper = s_Upper;
    addRequirements(s_Upper);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    s_Upper.setLeftShooter(0);
    s_Upper.setRightShooter(0);
    s_Upper.setIntake(0);
  }

  @Override
  public void execute() {
    s_Upper.setIntake(0);
    s_Upper.setLeftShooter(0);
    s_Upper.setRightShooter(0);
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished() {
    if (timer.get() >= 01) {
        return true;
    } else {
        return false;
    }
  }
}