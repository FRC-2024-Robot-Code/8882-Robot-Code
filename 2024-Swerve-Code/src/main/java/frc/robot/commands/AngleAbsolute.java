package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;

public class AngleAbsolute extends Command {

  Joystick control;
  PIDController anglePID;

  AngleShooter angle;

  double defaultAngle = 0.80;

  public AngleAbsolute(AngleShooter angle) {
    this.angle = angle;
    anglePID = new PIDController(3, 0, 0);

    addRequirements(angle);
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // if (control.getRawButton(Controle.kA)) {
    // // angle.setTarget(0.54);
    // } else if (control.getRawButton(Controle.kB)) {
    // angle.setTarget(0.30);
    // } else if (control.getRawButton(Controle.kY)) {
    // angle.setTarget(0.80);
    // }
    double output = anglePID.calculate(angle.getAbsolutePosition(), defaultAngle);
    angle.setSpeed(output);
  }

  @Override
  public void end(boolean interrupted) {
    angle.stop();
    angle.resetEncoder();
  }

  @Override
  public boolean isFinished() {
    return anglePID.atSetpoint();
  }
}
