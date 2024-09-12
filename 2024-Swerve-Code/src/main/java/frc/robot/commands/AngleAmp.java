package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AngleAmp extends Command {

     Angle angle;

     double setpoint;
     PIDController pid = new PIDController(1.5, 0, 0);

     public AngleAmp(Angle angle, double setpoint) {
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
          output = MathUtil.clamp(output, -3, 3);
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
