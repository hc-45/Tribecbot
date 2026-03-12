package com.stuypulse.robot.util.simulation;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

import com.stuypulse.robot.constants.Settings;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.IntakeSim;
import com.stuypulse.robot.subsystems.superstructure.hood.Hood;
import com.stuypulse.robot.subsystems.superstructure.hood.HoodSim;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter;
import com.stuypulse.robot.subsystems.superstructure.shooter.Shooter.ShooterState;
import com.stuypulse.robot.subsystems.superstructure.shooter.ShooterSim;
import com.stuypulse.robot.subsystems.superstructure.turret.Turret;
import com.stuypulse.robot.subsystems.superstructure.turret.TurretSim;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

public class Simulation {
    public static final Simulation instance;
    public final Arena2026Rebuilt ARENA;
    private final double SHOOTER_RATE_BPS;
    private final Notifier SHOOT_LOOP;

    static {
        instance = new Simulation();
    }

    public static Simulation getInstance() {
        return instance;
    }

    private final SwerveDriveSimulation mapleSimDrive;
    private final IntakeSimulation mapleSimIntake;
    private final Intake intakeSim;
    private final Shooter shooterSim;
    private final Hood hoodSim;
    private final Turret turretSim;
    
    /******************/
    /*** PUBLISHERS ***/
    /******************/
    private final NetworkTableInstance table;
    private final StructPublisher<Pose2d> drivetrain;
    private final StructArrayPublisher<SwerveModuleState> swerve;
    private final StructArrayPublisher<Pose3d> fuel;
    private final StructPublisher<Pose3d> intakePivot;
    private final StructPublisher<Pose3d> hopper;
    private final StructPublisher<Pose3d> intakeRollers;
    private final StructPublisher<Pose3d> shooter;

    private Simulation() {
        ARENA = new Arena2026Rebuilt(false);
        SHOOTER_RATE_BPS = 8;

        SHOOT_LOOP = new Notifier(this::updateShooting);
        SHOOT_LOOP.startPeriodic(1 / SHOOTER_RATE_BPS);

        intakeSim = IntakeSim.getInstance();
        shooterSim = ShooterSim.getInstance();
        hoodSim = HoodSim.getInstance();
        turretSim = TurretSim.getInstance();

        mapleSimDrive = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();
        
        this.configure();

        mapleSimIntake = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            mapleSimDrive,
            Meters.of(0.5),
            Meters.of(0.15),
            IntakeSimulation.IntakeSide.FRONT,
            15
        );

        table = NetworkTableInstance.getDefault();
        fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        drivetrain = table.getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
        swerve = table.getStructArrayTopic("AdvScope/SwerveStates", SwerveModuleState.struct).publish();
        intakePivot = table.getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
        hopper = table.getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
        intakeRollers = table.getStructTopic("AdvScope/IntakeRollerPose", Pose3d.struct).publish();
        shooter = table.getStructTopic("AdvScope/ShooterPose", Pose3d.struct).publish();
    }

    /**
     * <h3>Configures arena and gamepieces</h3>
     * <p>
     * This method overrides the arena instance and resets the field. Should only be
     * called once.
     * <strong>This method will reconstruct the drivetrain and erase anything stored on it.</strong>
     */
    public void configure() {
        ARENA.resetFieldForAuto();
        ARENA.addDriveTrainSimulation(mapleSimDrive);
        SimulatedArena.overrideInstance(ARENA);
        SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(
            new Translation2d(3.0, 2.0)
        ));
    }

    public void updateShooting() {
        if (shooterSim.getState() == ShooterState.SOTM || shooterSim.getState() == ShooterState.FOTM) {
            if (!mapleSimIntake.obtainGamePieceFromIntake()) return;

            Pose2d turretPose = CommandSwerveDrivetrain.getInstance().getTurretPose();
            ARENA.addPieceWithVariance(
                turretPose.getTranslation(),
                turretPose.getRotation(),
                Meters.of(0.5), // Settings.Superstructure.Turret.TURRET_HEIGHT
                MetersPerSecond.of(
                    (shooterSim.getRPM() * Settings.Superstructure.Shooter.FLYWHEEL_RADIUS * Math.PI) / 60 // conversion from RPM to linear velocity assuming no slip
                ),
                Degrees.of(hoodSim.getAngle().getDegrees()),
                0.0,
                0.01,
                0.01,
                0.1,
                0.05 // TODO: find real values
            );
            SmartDashboard.putNumber("Hood/SIM_ANGLE_TEMP", hoodSim.getAngle().getDegrees());
        };
    }

    private void setIntakeSim(boolean deployed) {
        if (deployed) {
            mapleSimIntake.startIntake();
        } else {
            mapleSimIntake.stopIntake();
        }
        SmartDashboard.putBoolean("Intake/MapleSim", deployed);
        SmartDashboard.putNumber("Intake/MapleSimPieces", mapleSimIntake.getGamePiecesAmount());
    }

    public synchronized void update() {
        if (mapleSimDrive != null) {
            setIntakeSim(
                intakeSim.pivotAtTolerance()
                    && intakeSim.getPivotState() 
                == IntakeSim.PivotState.DEPLOY
                    && intakeSim.getRollerState()
                == IntakeSim.RollerState.INTAKE
            );

            drivetrain.set(mapleSimDrive.getSimulatedDriveTrainPose());

            swerve.set(Arrays.stream(mapleSimDrive.getModules())
                .map(SwerveModuleSimulation::getCurrentState)
                .toArray(SwerveModuleState[]::new));
            fuel.set(ARENA.getGamePiecesArrayByType("Fuel"));

            intakePivot.set(
                new Pose3d(
                    0.2393388152,
                    0,
                    0.19685,
                    new Rotation3d(intakeSim.getPivotAngle().getRadians() - Math.toRadians(19) /* account for CAD roll offset */, 0, Math.toRadians(-90))
                )
            );

            double intakePivotEndX = 0.1093388152 + 0.2639822 * Math.cos(Math.toRadians(intakeSim.getPivotAngle().getDegrees()));
            hopper.set(
                new Pose3d(
                    intakePivotEndX - 0.38, // more offsetting
                    0,
                    0,
                    new Rotation3d(Math.toRadians(90), 0, Math.toRadians(90))
                )
            );

            intakeRollers.set(new Pose3d(
                intakePivotEndX, // offsets are slightly cooked, but they're close enough
                0,
                0.2212848 + 0.2639822 * Math.sin(Math.toRadians(intakeSim.getPivotAngle().getDegrees())),
                new Rotation3d(Math.toRadians(90), 0, Math.toRadians(90))
            ));

            shooter.set(new Pose3d(
                -0.1016,
                0.2032,
                0.3255608932, // offsets from origin in HAL 2026, scrambled around because export orientation was bad
                new Rotation3d(
                    Math.toRadians(90),
                    0,
                    turretSim.getAngle().getRadians() + Math.toRadians(-90) // add the CAD zero angle offset
                )
            ));
        }
    }
}
