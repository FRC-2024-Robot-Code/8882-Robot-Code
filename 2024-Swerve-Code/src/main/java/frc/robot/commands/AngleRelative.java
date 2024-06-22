package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AngleShooter;

public class AngleRelative extends Command {
     AngleShooter angle;
     Joystick control;
     PIDController pidController;

     double setpoint;

     public AngleRelative(AngleShooter angle, double setpoint) {
          this.angle = angle;
          this.setpoint = setpoint;
          pidController = new PIDController(1, 0, 0);

          addRequirements(angle);
     }

     @Override
     public void initialize() {
          // angle.resetEncoder();
     }

     @Override
     public void execute() {
          double outPut = pidController.calculate(angle.getPosition(), setpoint);

          angle.setSpeed(outPut);
     }

     @Override
     public void end(boolean interrupted) {

     }

     @Override
     public boolean isFinished() {
          return false;
     }
}
