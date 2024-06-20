package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class GyroLimelight extends Command {

  PIDController anglePIDController;

  SwerveSubsystem swerve;

  double setPoint;

  XboxController control = new XboxController(0);

  public GyroLimelight(SwerveSubsystem subsystem, double setPoint) {
    swerve = subsystem;
    this.setPoint = setPoint;
    anglePIDController = new PIDController(
        20,
        0,
        0);
    anglePIDController.enableContinuousInput(-54, 54);

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // double id =
    // NetworkTableInstance.getDefault().getTable("").getEntry("tid").getDouble(0);
    double outPut = anglePIDController.calculate(
        Math.toRadians(LimelightHelpers.getTX("")),
        Math.toRadians(setPoint));

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
