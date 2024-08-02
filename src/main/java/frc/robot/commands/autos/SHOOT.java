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

public class SHOOT extends Command {

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

  public SHOOT(Upper s_Upper) {
    this.s_Upper = s_Upper;
    addRequirements(s_Upper);
  }

  @Override
  public void initialize() {
    timer.reset();
  }

  @Override
  public void execute() { // time need to test
    if (Math.abs(s_Upper.getShooterRPM()) <= UpperConstants.SHOOTER_LEGAL_SPEED) {
    RobotConstants.upperState = UpperState.SHOOT;
    s_Upper.setElbow(-elbowPID.calculate(UpperStateMachine.elbowTarget.getRotations() - s_Upper.getElbowRotation()));
    s_Upper.setIntake(UpperStateMachine.intakeTarget.getRevPM() / UpperConstants.INTAKE_MAX_RPM);
    double output = LinearRegression.calculate(MapConstants.SHOOTER_RPM_TO_OUTPUT, UpperStateMachine.shooterTarget.getRevPM());
    s_Upper.setLeftShooter(output + shooterPID.calculate(s_Upper.getLeftShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));
    s_Upper.setRightShooter(output + shooterPID.calculate(s_Upper.getRightShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));
    } if (timer.get() >= 1.5 || Math.abs(s_Upper.getShooterRPM()) >= UpperConstants.SHOOTER_LEGAL_SPEED) {
    RobotConstants.upperState = UpperState.SPEAKER;
    s_Upper.setElbow(-elbowPID.calculate(UpperStateMachine.elbowTarget.getRotations() - s_Upper.getElbowRotation()));
    s_Upper.setIntake(UpperStateMachine.intakeTarget.getRevPM() / UpperConstants.INTAKE_MAX_RPM);
    double output = LinearRegression.calculate(MapConstants.SHOOTER_RPM_TO_OUTPUT, UpperStateMachine.shooterTarget.getRevPM());
    s_Upper.setLeftShooter(output + shooterPID.calculate(s_Upper.getLeftShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));
    s_Upper.setRightShooter(output + shooterPID.calculate(s_Upper.getRightShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));
    timer.start();
    }
  }

  @Override
  public void end(boolean interrupted){
    System.out.println("SHOOT end");
  }

  @Override
  public boolean isFinished() {
    if (timer.get() >= 3) {
      return true;
    } else {
      return false;
    }
  }
}