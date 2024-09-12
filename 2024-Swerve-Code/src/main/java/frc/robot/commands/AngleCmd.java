package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AngleCmd extends Command {

     Angle angle;

     double setpoint;
     PIDController pid = new PIDController(1.7, 0.025, 0);

     public AngleCmd(Angle angle, double setpoint) {
          this.angle = angle;
          this.setpoint = setpoint;

          addRequirements(angle);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          double output = pid.calculate(angle.getABSgyro(), setpoint);
          output = MathUtil.clamp(output, -1, 1);
          angle.setAngleSpeed(output);
     }

     @Override
     public void end(boolean interrupted) {
          angle.stopAngle();
     }

     @Override
     public boolean isFinished() {
          return false;
     }
}
