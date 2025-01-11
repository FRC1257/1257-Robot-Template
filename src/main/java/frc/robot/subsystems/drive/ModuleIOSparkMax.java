// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import java.util.Queue;
import static frc.robot.subsystems.drive.DriveConstants.*;
import static frc.robot.subsystems.drive.ModuleConstants.*;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final SparkMax driveSparkMax;
  private final SparkMax turnSparkMax;

  private final SparkMaxConfig driveConfig;
  private final SparkMaxConfig turnConfig;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AbsoluteEncoder turnAbsoluteEncoder;

  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private final double absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new SparkMax(kFrontLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kFrontLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = kFrontLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkMax = new SparkMax(kFrontRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kFrontRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = kFrontRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkMax = new SparkMax(kRearLeftDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kRearLeftTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = kBackLeftChassisAngularOffset; // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkMax = new SparkMax(kRearRightDrivingCanId, MotorType.kBrushless);
        turnSparkMax = new SparkMax(kRearRightTurningCanId, MotorType.kBrushless);
        absoluteEncoderOffset = kBackRightChassisAngularOffset; // MUST BE CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveConfig = new SparkMaxConfig();
    turnConfig = new SparkMaxConfig();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveConfig
      .smartCurrentLimit(kDrivingMotorCurrentLimit)
      .voltageCompensation(12.0)
      .idleMode(kDrivingMotorIdleMode);

    driveConfig
      .encoder
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(2);

    turnConfig
      .inverted(isTurnMotorInverted)
      .smartCurrentLimit(kTurningMotorCurrentLimit)
      .voltageCompensation(12.0);
    
    turnConfig
      .encoder
        .quadratureMeasurementPeriod(10)
        .quadratureAverageDepth(2);
    
    turnConfig
      .absoluteEncoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor)
        .inverted(ModuleConstants.kTurningEncoderInverted);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveConfig
      .signals
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Module.ODOMETRY_FREQUENCY));

    turnConfig
      .signals
        .primaryEncoderPositionPeriodMs((int) (1000.0 / Module.ODOMETRY_FREQUENCY));

    driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turnSparkMax.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnAbsoluteEncoder = turnSparkMax.getAbsoluteEncoder();

    driveEncoder.setPosition(0.0);

    turnRelativeEncoder.setPosition(0.0);
    
    timestampQueue = SparkMaxOdometryThread.getInstance().makeTimestampQueue();
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveEncoder.getPosition() / (kWheelDiameterMeters / 2);
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMeterPerSec = driveEncoder.getVelocity();
    inputs.driveVelocityRadPerSec = driveEncoder.getVelocity() / (kWheelDiameterMeters / 2);
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition = getTurnPosition();
    inputs.turnPosition = getTurnPosition();
    inputs.turnVelocityRadPerSec = turnAbsoluteEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value))
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value))
            .toArray(Rotation2d[]::new);
    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  public Rotation2d getTurnPosition() {
    double angle = turnAbsoluteEncoder.getPosition() - absoluteEncoderOffset;
    
    return Rotation2d.fromRadians(angle);
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    driveConfig.idleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    driveSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}