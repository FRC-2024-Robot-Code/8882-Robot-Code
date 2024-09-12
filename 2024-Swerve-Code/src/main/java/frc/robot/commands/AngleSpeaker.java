package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Angle;

public class AngleSpeaker extends Command {

     Angle angle;
     PIDController pid = new PIDController(1.7, 0.025, 0);
     InterpolatingDoubleTreeMap anglesMap = new InterpolatingDoubleTreeMap();
     double inter = 0;

     public AngleSpeaker(Angle angle) {
          this.angle = angle;

          anglesMap.put(2.1, 0.313);// .320
          anglesMap.put(2.25, 0.293);// .300
          anglesMap.put(2.38, 0.268);// .275
          anglesMap.put(2.54, 0.258);// .265

          anglesMap.put(2.68, 0.245);// .250
          anglesMap.put(2.86, 0.241);// .245
          anglesMap.put(3.0, 0.231);// .236
          anglesMap.put(3.1, 0.227);// .232
          anglesMap.put(3.37, 0.225);// .230

          addRequirements(angle);
     }

     @Override
     public void initialize() {
     }

     @Override
     public void execute() {
          inter = anglesMap.get(angle.getTz());
          if (inter > 1) {
               inter = 0.8;
          } else if (inter < 0.1) {
               inter = 0.5;
          }

          double output = pid.calculate(angle.getABSgyro(), inter);
          output = MathUtil.clamp(output, -4, 4);
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
