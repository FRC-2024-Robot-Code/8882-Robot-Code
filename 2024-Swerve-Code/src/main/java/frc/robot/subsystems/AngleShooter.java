package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PID;
import frc.robot.LimelightHelpers;

public class AngleShooter extends SubsystemBase {

  CANSparkMax angle1, angle2;
  DutyCycleEncoder absolutePIDencoder;
  RelativeEncoder encoder;

  PIDController anglePidController;
  PIDController alignPID;
  LimelightHelpers camera = new LimelightHelpers();
  InterpolatingDoubleTreeMap interpolating = new InterpolatingDoubleTreeMap();

  DoubleLogEntry setpointLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "angle/setpoint");

  double setpoint = 0;

  public AngleShooter() {
    anglePidController = new PIDController(PID.kP, PID.kI, PID.kD);
    alignPID = new PIDController(1, 0, 0);
    absolutePIDencoder = new DutyCycleEncoder(3);
    angle1 = new CANSparkMax(11, MotorType.kBrushless);
    angle2 = new CANSparkMax(12, MotorType.kBrushless);

    encoder = angle1.getEncoder();
    absolutePIDencoder.setDutyCycleRange(0.20, 0.99);

    angle2.setInverted(true);
    angle1.setInverted(true);

  }

  // public double getAngle() {
  // double inter = 0;

  // double[] get = LimelightHelpers.getTargetPose_RobotSpace("");

  // if (get.length >= 2) {
  // double tz = get[2];

  // inter = interpolating.get(tz);

  // if (inter > 1) {
  // inter = 0.8;
  // } else if (inter < 0.1) {
  // inter = 0.5;
  // }

  // }
  // return inter;
  // }

  private double getTz() {
    double[] get = LimelightHelpers.getTargetPose_RobotSpace("");

    return get.length >= 2 ? get[2] : 0.0;
  }

  public void stop() {
    angle1.stopMotor();
    angle2.stopMotor();
  }

  public double getPosition() {
    return Math.abs(encoder.getPosition());
  }

  public double getAbsolutePosition() {
    return absolutePIDencoder.getAbsolutePosition();
  }

  public void setTarget(double setPoint) {
    anglePidController.setSetpoint(setPoint);
    setpointLogEntry.append(setPoint);
  }

  public void resetEncoder() {
    encoder.setPosition(0);
  }

  public void setSpeed(double outPut) {
    angle1.set(outPut);
    angle2.set(-outPut);
  }

  public void align() {
    double defaultAngle = 0.8;
    while (!alignPID.atSetpoint()) {
      double speedAlign = alignPID.calculate(getAbsolutePosition(), defaultAngle);
      setSpeed(speedAlign);
    }
      stop();
      resetEncoder();
  }

  @Override
  public void periodic() {
    double outPut = anglePidController.calculate(getAbsolutePosition());

    outPut = MathUtil.clamp(outPut, -0.2, 0.2);

    // setSpeed(outPut);

    SmartDashboard.putNumber("Tz", getTz());
    SmartDashboard.putNumber("Position", getPosition());
    SmartDashboard.putNumber("ABS Position", getAbsolutePosition());
    SmartDashboard.putNumber("Setpoint", anglePidController.getSetpoint() * 360);
  }
}