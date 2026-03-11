package com.stuypulse.robot.util.simulation;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import com.stuypulse.robot.constants.Settings;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import com.stuypulse.robot.subsystems.intake.Intake;
import com.stuypulse.robot.subsystems.intake.Intake.RollerState;
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

import java.util.ArrayList;

import edu.wpi.first.units.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Angle;

public class Simulation {
    public static final Simulation instance;
    public final Arena2026Rebuilt ARENA;
    private double ballsPerSecond;

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
    private final StructPublisher<Pose3d> pivot;
    private final StructPublisher<Pose3d> hopper;

    private Simulation() {
        ARENA = new Arena2026Rebuilt(false);
        
        intakeSim = IntakeSim.getInstance();
        shooterSim = ShooterSim.getInstance();
        hoodSim = HoodSim.getInstance();
        turretSim = TurretSim.getInstance();

        mapleSimDrive = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();
        mapleSimIntake = IntakeSimulation.OverTheBumperIntake(
                "Fuel",
                mapleSimDrive,
                Meters.of(1.234),
                Meters.of(2.345),
                IntakeSimulation.IntakeSide.FRONT,
                67);

        table = NetworkTableInstance.getDefault();
        fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        drivetrain = table.getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
        swerve = table.getStructArrayTopic("AdvScope/SwerveStates", SwerveModuleState.struct).publish();
        pivot = table.getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
        hopper = table.getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
    }

    /**
     * <h3>Configures arena and gamepieces</h3>
     * <p>
     * This method overrides the arena instance and resets the field. Should only be
     * called once.
     */
    public void configure() {
        ARENA.resetFieldForAuto();
        ARENA.addDriveTrainSimulation(mapleSimDrive);
        SimulatedArena.overrideInstance(ARENA);
    }

    public void updateShootingProjectile(Arena2026Rebuilt arena) {
        if (shooterSim.getState() == ShooterState.SHOOT) {
            ballsPerSecond = 8;
            if (mapleSimIntake.obtainGamePieceFromIntake()) {
                return;
            }

            double speed = ShooterSim.getInstance().getRPM() * 2 * Math.PI / 60 * Settings.Superstructure.Shooter.FLYWHEEL_RADIUS;
            Translation2d robotPose = mapleSimDrive.getSimulatedDriveTrainPose().getTranslation();
        // we need the offset
            arena.addPieceWithVariance(

            );
        };
    }

    private void launchFuel() {
        RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
            CommandSwerveDrivetrain.getInstance().getPose().getTranslation(),
            new Translation2d(0.2, 0),
            CommandSwerveDrivetrain.getInstance().getChassisSpeeds(),
            mapleSimDrive.getSimulatedDriveTrainPose().getRotation().plus(turretSim.getAngle()),
            Meters.of(0.45),
            LinearVelocity.ofBaseUnits(shooterSim.getRPM() * 2 * Math.PI
                    * Settings.Superstructure.Shooter.FLYWHEEL_RADIUS, Units.MetersPerSecond),
            Angle.ofBaseUnits(hoodSim.getAngle().getRadians(), Units.Radians)
        );
        fuelOnFly
            .withTargetPosition(() -> FieldMirroringUtils.toCurrentAllianceTranslation(new Translation3d(0.25, 5.56, 2.3)))
            .withTargetTolerance(new Translation3d(0.6096, 0.5334, 0.000508)) // 24 in x 21 in x 0.02 in
            .withHitTargetCallBack(() -> {})
            .withTouchGroundHeight(0.1)
            .enableBecomesGamePieceOnFieldAfterTouchGround();
        // https://github.com/Shenzhen-Robotics-Alliance/maple-sim/blob/main/CLAUDE.md
        ARENA.addGamepieceProjectile(fuelOnFly);
    }

    private void toggleIntakeSim(boolean deployed) {
        if (deployed) 
            mapleSimIntake.startIntake();
        else
            mapleSimIntake.stopIntake();
    }

    public synchronized void update() {
        if (mapleSimDrive != null) {
            drivetrain.set(mapleSimDrive.getSimulatedDriveTrainPose());
            ArrayList<SwerveModuleState> states = new ArrayList<>();
            for (SwerveModuleSimulation module : mapleSimDrive.getModules()) {
                states.add(module.getCurrentState());
            }
            swerve.set(states.toArray(new SwerveModuleState[0]));
            fuel.set(ARENA.getGamePiecesArrayByType("Fuel"));
            pivot.set(
                new Pose3d(
                    0,
                    0,
                    0,
                    new Rotation3d(0, 0, 0)
                )
            );
            hopper.set(
                new Pose3d(
                    0,
                    0,
                    0,
                    new Rotation3d(Math.toRadians(90), 0, Math.toRadians(90))
                )
            );

            toggleIntakeSim(
                intakeSim.pivotAtTolerance()
                    && intakeSim.getPivotState() 
                == IntakeSim.PivotState.DEPLOY
            );
        }
    }
}
