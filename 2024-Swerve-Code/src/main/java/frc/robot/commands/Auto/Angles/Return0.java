package frc.robot.commands.Auto.Angles;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class Return0 extends Command {
     Angle angle;
     double setpoint;
     PIDController pid = new PIDController(0.8, 0, 0);

     public Return0(Angle angle) {

          this.angle = angle;

          addRequirements(angle);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          double output = pid.calculate(angle.getABSgyro(), 0.90);
          output = MathUtil.clamp(output, -4, 4);
          angle.setAngleSpeed(output);
     }

     @Override
     public void end(boolean interrupted) {
          angle.stopAngle();
          this.cancel();
     }

     @Override
     public boolean isFinished() {
          return pid.atSetpoint();

     }
}
