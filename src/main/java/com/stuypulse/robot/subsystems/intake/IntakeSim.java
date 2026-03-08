package com.stuypulse.robot.subsystems.intake;

import java.util.Optional;

import com.stuypulse.robot.RobotContainer.EnabledSubsystems;
import com.stuypulse.robot.constants.Gains;
import com.stuypulse.robot.constants.Settings;
import com.stuypulse.robot.util.SysId;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounce;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class IntakeSim extends Intake {
    private final FlywheelSim intakeRollerMotor;

    private final SingleJointedArmSim pivot;
    private final PIDController pivotController;
    private final TrapezoidProfile pivotProfile;
    private double pivotZeroOffsetRads;
    private Optional<Double> pivotVoltageOverride;
    private BStream pivotStalling;
    private double pivotVoltage;

    private final StructPublisher<Pose3d> pivotPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
    private final StructPublisher<Pose3d> hopperPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
    
    public IntakeSim() {
        pivotZeroOffsetRads = 0.0;

        intakeRollerMotor = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2),
                0.1,
                Settings.Intake.GEAR_RATIO
            ),
            DCMotor.getKrakenX60(2)
        );

        pivotController = new PIDController(
            Gains.Intake.Pivot.kP,
            Gains.Intake.Pivot.kI,
            Gains.Intake.Pivot.kD
        );

        pivotProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Settings.Intake.PIVOT_MAX_VEL_STOW.getRadians(),
                Settings.Intake.PIVOT_MAX_ACCEL_STOW.getRadians()
            )
        );

        pivot = new SingleJointedArmSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1),
                0.1,
                Settings.Intake.GEAR_RATIO
            ),
            DCMotor.getKrakenX60(1),
            Settings.Intake.GEAR_RATIO,
            Settings.Intake.PIVOT_ARM_LENGTH_METERS,
            Settings.Intake.PIVOT_MIN_ANGLE.getRadians(),
            Settings.Intake.PIVOT_MAX_ANGLE.getRadians(),
            true,
            Settings.Intake.PIVOT_STOW_ANGLE.getRadians());

        pivotVoltageOverride = Optional.empty();
        
        pivotStalling = BStream.create(
                () -> Math.abs(pivot.getCurrentDrawAmps()) > Settings.Intake.STALL_CURRENT_LIMIT)
                .filtered(new BDebounce.Both(Settings.Intake.STALL_DEBOUNCE));

        pivotVoltage = 0.0;
    }

    @Override
    public Rotation2d getPivotAngle() {
        return new Rotation2d(pivot.getAngleRads() - pivotZeroOffsetRads);
    }

    
    @Override
    public void zeroPivotStowed() {
        pivotZeroOffsetRads = pivot.getAngleRads() - Settings.Intake.PIVOT_STOW_ANGLE.getRadians();
    }

    @Override
    public void zeroPivotDeployed() {
        pivotZeroOffsetRads = pivot.getAngleRads() - Settings.Intake.PIVOT_DEPLOY_ANGLE.getRadians();
    }

    @Override
    public boolean pivotStalling() {
        return pivotStalling.get();
    }

    @Override
    public boolean pivotAtTolerance() {
        return Math.abs(
            (getPivotAngle().getRotations()) - getPivotState().getTargetAngle().getRotations()) 
                < Settings.Intake.PIVOT_ANGLE_TOLERANCE.getRotations();
    }

    public double getPivotVoltage() {
        return this.pivotVoltage;
    }
    @Override
    public void periodic() {
        super.periodic();
        PivotState pivotState = getPivotState();
        pivotVoltage = pivotController.calculate(pivot.getAngleRads(), getPivotState().getTargetAngle().getRadians());

        double rollerVoltage = getRollerState().getTargetDutyCycle() * Settings.Intake.CURRENT_LIMIT;

        if (EnabledSubsystems.INTAKE.get()) {
            if (pivotVoltageOverride.isPresent()) {
                pivot.setInputVoltage(pivotVoltageOverride.get());
            } else {
                // PIVOT
                if (pivotState == PivotState.DEPLOY
                        && getPivotAngle().getDegrees() <= Settings.Intake.ARBITRARY_VOLTAGE_THRESHOLD.getDegrees()) {
                    pivot.setInputVoltage(-Settings.Intake.PUSHDOWN_VOLTAGE); // applying 3 volts
                } else if (pivotState == PivotState.DIGESTION_DOWN || pivotState == PivotState.DIGESTION_UP) {
                    TrapezoidProfile.State profileState = pivotProfile.calculate(
                        Settings.DT,
                        new TrapezoidProfile.State(getPivotAngle().getRadians(), pivot.getVelocityRadPerSec()),
                        new TrapezoidProfile.State(pivotState.getTargetAngle().getRadians(), 0)
                    );
                    pivot.setInputVoltage(pivotController.calculate(pivot.getAngleRads(), profileState.position)); // TODO: verify motion profile works
                } else {
                    pivot.setInputVoltage(pivotVoltage);
                }

                // ROLLERS
                if (pivotState == PivotState.DEPLOY
                        && getPivotAngle().getDegrees() <= Settings.Intake.THRESHOLD_TO_START_ROLLERS.getDegrees()) {
                    intakeRollerMotor.setInputVoltage(rollerVoltage);
                } else {
                    intakeRollerMotor.setAngularVelocity(0);
                }
            }
        } else {
            // stop motors
            pivot.setState(getPivotAngle().getRadians(), 0);
            intakeRollerMotor.setAngularVelocity(0);
        }

        pivot.update(Settings.DT);
        intakeRollerMotor.update(Settings.DT);

        pivotPublisher.set(
            new Pose3d(
                0,
                0,
                0,
                new Rotation3d(
                    0.0,
                    0.0,
                    0.0
                )
            )
        );

        hopperPublisher.set(
            new Pose3d(
                0,
                0,
                0,
                new Rotation3d(Math.toRadians(0), 0, Math.toRadians(0))
            )
        );

        SmartDashboard.putNumber("Intake/rollerVoltage", rollerVoltage);
        SmartDashboard.putNumber("Intake/pivotVoltage", pivotVoltage);
        SmartDashboard.putNumber("Intake/pivotAngle", Math.toDegrees(pivot.getAngleRads()));
        SmartDashboard.putNumber("Intake/pivotTarget", getPivotState().getTargetAngle().getDegrees());
    }

    @Override
    public void setPivotVoltageOverride(Optional<Double> voltage) {
        this.pivotVoltageOverride = voltage;
    }

    @Override
    public SysIdRoutine getPivotSysIdRoutine() {
        return SysId.getRoutine(
                2.0,
                6.0,
                "Intake Pivot",
                voltage -> setPivotVoltageOverride(Optional.of(voltage)),
                () -> getPivotAngle().getRotations(),
                () -> pivot.getVelocityRadPerSec(),
                () -> getPivotVoltage(),
                getInstance());
    }
}