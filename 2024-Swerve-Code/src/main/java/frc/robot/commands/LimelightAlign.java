package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Swerve;

public class LimelightAlign extends Command {
     PIDController gyroPID;
     Swerve swerve;
     double setpoint;
     XboxController control;

     public LimelightAlign(Swerve swerve, XboxController control) {
          this.swerve = swerve;
          this.control = control;
          gyroPID = new PIDController(3, 0, 0);

          gyroPID.enableContinuousInput(-54, 54);
          // gyroPID.setTolerance(2);

          addRequirements(swerve);
     }

     @Override
     public void execute() {

          // double id = NetworkTableInstance.getDefault().getTable("").getEntry("tid").getDouble(0);
          double outPut = gyroPID.calculate(Math.toDegrees(LimelightHelpers.getTX("")),
                    Math.toDegrees(0));

          swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, outPut));
     }

     @Override
     public void end(boolean interrupted) {
          swerve.setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
     }

     @Override
     public boolean isFinished() {
          if (control.getXButtonReleased()) {
               return true;
          }
          return false;
     }
}
