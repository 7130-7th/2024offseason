package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.FSLib.math.LinearRegression;
import frc.FSLib.math.PID;
import frc.FSLib.math.simplePID;
import frc.FSLib.util.AngularVelocity;
import frc.robot.Constants.UpperConstants;
import frc.robot.Constants.UpperStateMachine;
import frc.robot.Constants.UpperStateMachine.UpperState;
import frc.robot.Constants.MapConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Upper;

public class TeleopUpper extends Command {

    private final Upper s_Upper;
    private final XboxController controller;

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

    public TeleopUpper(Upper s_Upper, XboxController controller) {
        this.s_Upper = s_Upper;
        this.controller = controller;
        addRequirements(s_Upper);
    }

    @Override
    public void execute() {

        if (controller.getYButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.GROUND ? UpperState.DEFAULT : UpperState.GROUND;
        if (controller.getXButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.AMP ? UpperState.DEFAULT : UpperState.AMP;
        if (controller.getAButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.SPEAKER ? UpperState.DEFAULT : UpperState.SPEAKER;
        if (controller.getRightBumperPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.ENDGAME ? UpperState.DEFAULT : UpperState.ENDGAME;
        if(controller.getRightTriggerAxis() > 0.8) RobotConstants.upperState = UpperState.SHOOT;
        if(controller.getRightTriggerAxis() < 0.8 && RobotConstants.upperState == UpperState.SHOOT) RobotConstants.upperState = UpperState.DEFAULT;
        if (controller.getBButtonPressed()) RobotConstants.upperState = RobotConstants.upperState == UpperState.SHOOT ? UpperState.DEFAULT : UpperState.SHOOT;

        if (s_Upper.hasNote()) {UpperConstants.INTAKE_GROUND_SPEED = AngularVelocity.fromRevPM(0);} 
        else {UpperConstants.INTAKE_GROUND_SPEED = AngularVelocity.fromRevPM(-1800);}

        s_Upper.setElbow(-elbowPID.calculate(UpperStateMachine.elbowTarget.getRotations() - s_Upper.getElbowRotation()));
        s_Upper.setIntake(UpperStateMachine.intakeTarget.getRevPM() / UpperConstants.INTAKE_MAX_RPM);
        double output = LinearRegression.calculate(MapConstants.SHOOTER_RPM_TO_OUTPUT, UpperStateMachine.shooterTarget.getRevPM());
        s_Upper.setLeftShooter(output + shooterPID.calculate(s_Upper.getLeftShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));
        s_Upper.setRightShooter(output + shooterPID.calculate(s_Upper.getRightShooterRPM(), UpperStateMachine.shooterTarget.getRevPM()));
    }
}
