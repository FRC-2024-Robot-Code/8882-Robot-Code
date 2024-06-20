package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PID;
import frc.robot.LimelightHelpers;

public class AngleShooter extends SubsystemBase {

  CANSparkMax angle1, angle2;
  DutyCycleEncoder pidEncoder;
  PIDController anglePidController;
  LimelightHelpers camera = new LimelightHelpers();
  InterpolatingDoubleTreeMap interpolating = new InterpolatingDoubleTreeMap();

  public AngleShooter() {
    anglePidController = new PIDController(PID.kP, PID.kI, PID.kD);
    pidEncoder = new DutyCycleEncoder(0);
    angle1 = new CANSparkMax(11, MotorType.kBrushless);
    angle2 = new CANSparkMax(12, MotorType.kBrushless);
    // pidEncoder.setPositionOffset(0.51);
    pidEncoder.setDutyCycleRange(0.20, 0.99);

    angle2.setInverted(true);
    angle1.setInverted(true);

  }

  public double getAngle() {
    double inter = 0;

    double[] get = LimelightHelpers.getTargetPose_RobotSpace("");

    if (get.length >= 2) {
      double tz = get[2];

      inter = interpolating.get(tz);

      if (inter > 1) {
        inter = 0.8;
      } else if (inter < 0.1) {
        inter = 0.5;
      }

    }
    return inter;
  }

  private double getTz() {
    double[] get = LimelightHelpers.getTargetPose_RobotSpace("");

    return get.length >= 2 ? get[2] : 0.0;
  }

  public void stop() {
    angle1.stopMotor();
    angle2.stopMotor();
  }

  public double getPosition() {
    return pidEncoder.getAbsolutePosition();
  }

  public void setTarget(double setPoint) {
    anglePidController.setSetpoint(setPoint);
  }

  public void setSpeed(double outPut) {
    angle1.set(outPut);
    angle2.set(-outPut);
  }

  @Override
  public void periodic() {
    double outPut = anglePidController.calculate(getPosition());

    outPut = MathUtil.clamp(outPut, -0.2, 0.2);

    setSpeed(outPut);

    SmartDashboard.putNumber("Position", getPosition());
    SmartDashboard.putNumber("Angle For Shoot", getAngle());
    SmartDashboard.putNumber("Setpoint", anglePidController.getSetpoint() * 360);
    SmartDashboard.putNumber("tz", getTz());
    SmartDashboard.putNumber("Velocity Angle", outPut);
  }
}