// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Dimensoes;
import frc.robot.Constants.Tracao;
import frc.robot.LimelightHelpers;
import frc.robot.commands.Auto.ConfigAuto;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;

/**
 * Classe de subsistema onde fazemos a ponte do nosso código para YAGSL
 */
public class Swerve extends SubsystemBase {
  public SwerveDrive swerveDrive;

  // Objeto global autônomo
  ConfigAuto autonomo;

  private Field2d field2d = new Field2d();

  // Método construtor da classe
  public Swerve(File directory) {
    // Seta a telemetria como nível mais alto
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    // Acessa os arquivos do diretório .JSON
    try {

      swerveDrive = new SwerveParser(directory).createSwerveDrive(Tracao.MAX_SPEED,
          Dimensoes.angleConversion, Dimensoes.driveConversion);

    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    autonomo = new ConfigAuto(this);

    autonomo.setupPathPlanner();

    // SmartDashboard.putData(field2d);

  }

  @Override
  public void periodic() {
    // Dentro da função periódica atualizamos nossa odometria
    visionUpdateOdometry();
    swerveDrive.updateOdometry();
    SmartDashboard.putNumber("Distance to Goal", getDistancetoGoal());
    // SmartDashboard.putNumber("Odometry Y", getOdometryY());
  }

  // Swerve
  public void chassiAim(double spotToAim) {
    double angularVelocity = swerveDrive.swerveController.headingCalculate(getHeading().getRadians(), spotToAim);
    swerveDrive.drive(new ChassisSpeeds(0, 0, angularVelocity));
  }

  public double getAngleToAimToSpeaker() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      double angleDegree = getPose().getTranslation().minus(new Translation2d(16.45, 5.55)).getAngle().getRadians();
      return angleDegree;
    } else {
      double angleDegree = getPose().getTranslation().minus(new Translation2d(0.0, 5.55)).getAngle().getRadians();
      return angleDegree;
    }
  }

  public void aimToSpeaker() {
    chassiAim(getAngleToAimToSpeaker());
  }

  public double getOdometryX() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return Math.abs(getPose().getTranslation().minus(new Translation2d(16.45, 5.5)).getX());
    } else {
      return getPose().getTranslation().minus(new Translation2d(0.0, 5.5)).getX();
    }
  }

  public double getOdometryY() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      return getPose().getTranslation().minus(new Translation2d(16.45, 5.5)).getY();
    } else {
      return getPose().getTranslation().minus(new Translation2d(0.0, 5.5)).getY();
    }
  }

  public double getDistancetoGoal() {
    double distance = Math.sqrt((getOdometryX() * getOdometryX()) + (getOdometryY() * getOdometryY()));
    return distance;
  }

  public void visionUpdateOdometry() {
    if (!DriverStation.isAutonomous()) {
      boolean doRejectUpdate = false;
      // Ela pega a orientação do robô em relação a odometry
      LimelightHelpers.SetRobotOrientation("",
          swerveDrive.getOdometryHeading().getDegrees(),
          0, 0, 0, 0, 0);
      // Ela é a pose estimate da classe
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
      // if our angular velocity is greater than 720 degrees per second, ignore vision
      // updates
      if (Math.abs(Math.toDegrees(getRobotVelocity().omegaRadiansPerSecond)) > 200) {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,
            .7, 9999999));
        swerveDrive.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
      field2d.setRobotPose(mt2.pose);
    }

    // LimelightHelpers.PoseEstimate mt1 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    // boolean doRejectUpdate = false;
    // if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
    // if (mt1.rawFiducials[0].ambiguity > .7) {
    // doRejectUpdate = true;
    // }
    // if (mt1.rawFiducials[0].distToCamera > 3) {
    // doRejectUpdate = true;
    // }
    // }
    // if (mt1.tagCount == 0) {
    // doRejectUpdate = true;
    // }

    // if (!doRejectUpdate) {
    // swerveDrive.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,
    // .5, 9999999));
    // swerveDrive.addVisionMeasurement(
    // mt1.pose,
    // mt1.timestampSeconds);
    // }
  }

  public Measure<Distance> distanceToTarget(Translation2d target) {
    Translation2d robotPos = this.swerveDrive.getPose().getTranslation();
    double measured = robotPos.getDistance(target);
    return edu.wpi.first.units.Units.Meters.of(measured);
  }

  public void setHeadingCorrection(boolean headingCorrection) {
    swerveDrive.setHeadingCorrection(headingCorrection);
  }

  public Rotation2d rotationToTarget(Translation2d target) {
    Translation2d robotPos = this.swerveDrive.getPose().getTranslation();
    double targetRadians = Math.atan2(target.getY() - robotPos.getY(), target.getX() - robotPos.getX());
    double targetDegrees = Math.toDegrees(targetRadians);
    return new Rotation2d(targetDegrees);
  }

  // Função drive que chamamos em nossa classe de comando Teleoperado
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation, rotation, fieldRelative, false);
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  // Função para obter a velocidade desejada a partir dos inputs do gamepad
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, double headingX, double headingY) {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, headingX, headingY,
        getHeading().getRadians());
  }

  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput) {
    return swerveDrive.swerveController.getTargetSpeeds(xInput, yInput, 0, 0, 0, Tracao.MAX_SPEED);
  }

  // Função que retorna a posição do robô (translação e ângulo), (Usado no
  // autônomo)
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  // Retorna a velocidade relativa ao campo
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  // Retorna a configuração do swerve
  public SwerveDriveConfiguration getSwerveDriveConfiguration() {
    return swerveDrive.swerveDriveConfiguration;
  }

  // Retorna o objeto de controle, o qual é usado para acessar as velocidades
  // máximas por exemplo
  public SwerveController getSwerveController() {
    return swerveDrive.getSwerveController();
  }

  // Ângulo atual do robô
  public Rotation2d getHeading() {
    return swerveDrive.getYaw();
  }

  // Reseta a odometria para uma posição indicada (Usado no autônomo)
  public void resetOdometry(Pose2d posicao) {
    swerveDrive.resetOdometry(posicao);
  }

  // Seta a velocidade do chassi (Usado no autônomo)
  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void setMotorBrake(boolean brake) {
    swerveDrive.setMotorIdleMode(brake);
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  public ChassisSpeeds discretize(ChassisSpeeds speeds) {
    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * Tracao.dt,
        speeds.vyMetersPerSecond * Tracao.dt,
        new Rotation2d(speeds.omegaRadiansPerSecond * Tracao.dt * Tracao.constantRotation));
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / Tracao.dt), (twist.dy / Tracao.dt), (speeds.omegaRadiansPerSecond));
  }

  public Command getAutonomousCommand(String pathName, boolean alliance, boolean setOdomToStart) {

    return new PathPlannerAuto(pathName);
  }
}