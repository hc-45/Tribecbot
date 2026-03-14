/************************ PROJECT TRIBECBOT *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved.    */
/* Use of this source code is governed by an MIT-style license    */
/* that can be found in the repository LICENSE file.              */
/*******************************************************************/

package com.stuypulse.robot.util.simulation;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

import com.stuypulse.robot.constants.Field;
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
import com.stuypulse.robot.util.superstructure.InterpolationCalculator;
import com.stuypulse.robot.util.superstructure.InterpolationCalculator.InterpolatedShotInfo;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Simulation {
    public static final Simulation instance;

    static {
        instance = new Simulation();
    }

    public static Simulation getInstance() {
        return instance;
    }

    public final Arena2026Rebuilt ARENA;
    private final Notifier SHOOT_LOOP;

    /*******************************/
    /*** SUBSYSTEM SIM INSTANCES ***/
    /*******************************/

    private final SwerveDriveSimulation mapleSimDrive;
    private final IntakeSimulation mapleSimIntake;
    private final Intake intakeSim;
    private final Shooter shooterSim;
    private final Hood hoodSim;
    private final Turret turretSim;

    /********************************/
    /*** NETWORKTABLES PUBLISHERS ***/
    /********************************/

    private final StructPublisher<Pose2d> drivetrain;
    private final StructArrayPublisher<SwerveModuleState> swerve;
    private final StructPublisher<ChassisSpeeds> chassis;
    private final StructArrayPublisher<Pose3d> fuel;
    private final StructPublisher<Pose3d> intakePivot;
    private final StructPublisher<Pose3d> hopper;
    private final StructPublisher<Pose3d> intakeRollers;
    private final StructPublisher<Pose3d> shooter;

    private Simulation() {
        intakeSim = IntakeSim.getInstance();
        shooterSim = ShooterSim.getInstance();
        hoodSim = HoodSim.getInstance();
        turretSim = TurretSim.getInstance();
        mapleSimDrive = CommandSwerveDrivetrain.getInstance().getMapleSimDrive();

        ARENA = new Arena2026Rebuilt(false);
        this.configure(); // override instance before dependencies use it

        mapleSimIntake = createIntakeSimulation();
        mapleSimIntake.addGamePiecesToIntake(SimulationConstants.Hopper.FUEL_CAPACITY);

        SHOOT_LOOP = new Notifier(this::updateShooting);
        SHOOT_LOOP.startPeriodic(1.0 / SimulationConstants.Shooter.BPS * 2); // multiply by 2 to account for the delaying in updateShooting

        NetworkTableInstance table = NetworkTableInstance.getDefault();
        drivetrain = table.getStructTopic("AdvScope/DTPose", Pose2d.struct).publish();
        swerve = table.getStructArrayTopic("AdvScope/SwerveStates", SwerveModuleState.struct).publish();
        chassis = table.getStructTopic("AdvScope/ChassisSpeeds", ChassisSpeeds.struct).publish();
        fuel = table.getStructArrayTopic("AdvScope/FuelPoses", Pose3d.struct).publish();
        intakePivot = table.getStructTopic("AdvScope/IntakePose", Pose3d.struct).publish();
        hopper = table.getStructTopic("AdvScope/HopperPose", Pose3d.struct).publish();
        intakeRollers = table.getStructTopic("AdvScope/IntakeRollerPose", Pose3d.struct).publish();
        shooter = table.getStructTopic("AdvScope/ShooterPose", Pose3d.struct).publish();
    }

    /*************/
    /*** SETUP ***/
    /*************/

    /**
     * <h2>Registers the drivetrain with the arena</h2>
     * <p>Sets the arena as the active {@link SimulatedArena} instance. Should only be called once during construction.
     */
    public void configure() {
        ARENA.resetFieldForAuto();
        ARENA.addDriveTrainSimulation(mapleSimDrive);
        SimulatedArena.overrideInstance(ARENA);
    }

    /**
     * <h2>Creates an intake simulation</h2>
     * <p>Abstracts the creation of an {@link IntakeSimulation} instance.
     */
    private IntakeSimulation createIntakeSimulation() {
        return IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            mapleSimDrive,
            Meters.of(SimulationConstants.Intake.INTAKE_WIDTH),
            Meters.of(SimulationConstants.Intake.INTAKE_LENGTH),
            IntakeSimulation.IntakeSide.FRONT,
            SimulationConstants.Hopper.FUEL_CAPACITY
        );
    }

    /**********************/
    /*** SHOOTING LOGIC ***/
    /**********************/

    /**
     * <h2>Evaluates shooting viability</h2>
     * <p>Abstracts the checks done before shooting gamepieces
     * @return true if a shot should be attempted, false if not
     */
    private boolean shotConditionsSatisfied() {
        boolean shooterReady = shooterSim.getState() == ShooterState.SOTM
                || shooterSim.getState() == ShooterState.FOTM;
        return shooterReady && !turretSim.isWrapping() && hoodSim.atTolerance();
    }

    /**
     * <h2>Shoots fuel into the hub</h2>
     * Called periodically by the SHOOT_LOOP {@link Notifier} at a rate of {@link SimulationConstants.Shooter#BPS} to mimic the actual robot's shooting rate.
     * Uses a game piece from the intake and launches at the hub if shot conditions are met.
     */
    private void updateShooting() {
        Timer.delay(Math.abs(Arena2026Rebuilt.randomInRange(0.5)));
        if (!shotConditionsSatisfied())
            return;
        if (!mapleSimIntake.obtainGamePieceFromIntake())
            return;

        Pose2d turretPose = CommandSwerveDrivetrain.getInstance().getTurretPose();
        InterpolatedShotInfo shot = InterpolationCalculator.interpolateShotInfo(
            turretPose, Field.getHubPose());

        // hood angle works when its done like this for some reason 😂
        double launchPitchDeg = 90.0 - shot.targetHoodAngle().getDegrees();

        robotRelativeAddPieceWithVariance(
            turretPose.getTranslation(),
            turretPose.getRotation(),
            Meters.of(0.6),
            MetersPerSecond.of(SimulationConstants.Shooter.rpmToMps(shooterSim.getRPM())),
            Degrees.of(launchPitchDeg),
            SimulationConstants.LAUNCH_X_VARIANCE,
            SimulationConstants.LAUNCH_Y_VARIANCE,
            SimulationConstants.LAUNCH_YAW_VARIANCE,
            SimulationConstants.LAUNCH_SPEED_VARIANCE,
            SimulationConstants.LAUNCH_PITCH_VARIANCE
        );

        SmartDashboard.putNumber("Hood/SIM_ANGLE_TEMP", hoodSim.getAngle().getDegrees());
    }

    /**
     * <h2>Extension of {@link Arena2026Rebuilt#addPieceWithVariance} that uses chassis speeds</h2>
     * <p>Adds a game piece too the arena with a certain random variance.
     * Param docs taken from {@link Arena2026Rebuilt#addPieceWithVariance}
     *
     * @param info the info of the game piece
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the game piece
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's
     *     frame of reference
     * @param chassisSpeedsFieldRelative the field-relative velocity of the robot chassis when launching the game piece,
     *     influencing the initial velocity of the game piece
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the game piece when launched, i.e., the height of the shooter from the
     *     ground
     * @param launchingSpeed the speed at which the game piece is launch
     * @param shooterAngle the pitch angle of the shooter when launching
     * @param xVariance The max amount of variance that should be added too the x coordinate of the game piece.
     * @param yVariance The max amount of variance that should be added too the y coordinate of the game piece.
     * @param yawVariance The max amount of variance that should be added too the yaw of the game piece.
     * @param speedVariance The max amount of variance that should be added too the speed of the game piece.
     * @param pitchVariance The max amount of variance that should be added too the pitch of the game piece.
     * @param target The target of the gamepiece
     */
    private void robotRelativeAddPieceWithVariance(
        Translation2d piecePose,
        Rotation2d yaw,
        Distance height,
        LinearVelocity speed,
        Angle pitch,
        double xVariance,
        double yVariance,
        double yawVariance,
        double speedVariance,
        double pitchVariance) {
        ARENA.addGamePieceProjectile(
            new RebuiltFuelOnFly(
                piecePose.plus(new Translation2d(Arena2026Rebuilt.randomInRange(xVariance), Arena2026Rebuilt.randomInRange(yVariance))),
                new Translation2d(),
                mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                yaw.plus(Rotation2d.fromDegrees(Arena2026Rebuilt.randomInRange(yawVariance))),
                height,
                speed.plus(MetersPerSecond.of(Arena2026Rebuilt.randomInRange(speedVariance))),
                Degrees.of(pitch.in(Degrees) + Arena2026Rebuilt.randomInRange(pitchVariance))
            )
        );
    }

    /**********************/
    /*** INTAKE CONTROL ***/
    /**********************/

    /**
     * <h2>State baesd intake toggler</h2>
     * <p>Starts or stops the intake simulation based on whether the subsystem sims are deployed and intaking
     */
    private void updateIntakeSim() {
        if (intakeSim.pivotAtTolerance() && intakeSim.getPivotState() == IntakeSim.PivotState.DEPLOY && intakeSim.getRollerState() == IntakeSim.RollerState.INTAKE) {
            mapleSimIntake.startIntake();
        } else {
            mapleSimIntake.stopIntake();
        }
    }

    /*************************/
    /*** POSE CALCULATIONS ***/
    /*************************/

    /**
     * <h2>Returns the 3D pose of the intake pivot joint</h2>
     * <p>Accounts for the CAD roll offset
     */
    private Pose3d getIntakePivotPose() {
        return SimulationConstants.Intake.PIVOT_OFFSETS.withRotation(new Rotation3d(
            0,
            Math.toRadians(180)-intakeSim.getPivotAngle().getRadians(),
            0
        ));
    }

    /**
     * <h2>Returns the X coord of the end of the intake arm for the current pivot angle</h2>
     * <p>Shared between the hopper and roller pose calculations to avoid recalculation
     */
    private double getIntakeArmEndX() {
        return SimulationConstants.Intake.PIVOT_END_X
            + SimulationConstants.Intake.PIVOT_ARM_LENGTH
            * Math.cos(Math.toRadians(intakeSim.getPivotAngle().getDegrees()));
    }

    /**
     * <h2>Calculates the pose of the extendable part of the hopper</h2>
     * <p>Returns the moved pose based on the intake pivot's movement
     * @param armEndX the getd arm end X from {@link #getIntakeArmEndX()}
     */
    private Pose3d getHopperPose(double armEndX) {
        return SimulationConstants.Hopper.OFFSETS.applyToPose3d(new Pose3d(armEndX, 0, 0, new Rotation3d()));
    }

    /**
     * <h2>Calculates the pose of the intake rollers</h2>
     * <p>Uses the end x of the pivot arm to computate the roller position
     * @param armEndX the arm end X from {@link #getIntakeArmEndX()}
     */
    private Pose3d getIntakeRollerPose(double armEndX) {
        return SimulationConstants.Intake.ROLLER_OFFSETS.applyToPose3d(new Pose3d(
            armEndX,
            0,
            SimulationConstants.Intake.PIVOT_ARM_LENGTH * Math.sin(Math.toRadians(intakeSim.getPivotAngle().getDegrees())),
            new Rotation3d()
        ));
    }

    /**
     * <h2>Calculates the pose of the turret & shooter</h2>
     * <p>Returns calculated pose of the turret shooter complex with offsets applied
     */
    private Pose3d getShooterPose() {
        return SimulationConstants.Shooter.OFFSETS.withRotation(
            new Rotation3d(0, 0, turretSim.getAngle().getRadians())
        );
    }

    /************************/
    /*** MAIN UPDATE LOOP ***/
    /************************/

    /**
     * <h2>Publishes and updates all data</h2>
     * <p>Updates every subsystem's NetworkTables entry and prompts an update of the {@link IntakeSimulation} instance
     */
    public synchronized void update() {
        if (mapleSimDrive == null) return;

        updateIntakeSim();

        drivetrain.set(mapleSimDrive.getSimulatedDriveTrainPose());
        swerve.set(Arrays.stream(mapleSimDrive.getModules())
            .map(SwerveModuleSimulation::getCurrentState)
            .toArray(SwerveModuleState[]::new));
        chassis.set(mapleSimDrive.getDriveTrainSimulatedChassisSpeedsFieldRelative());

        fuel.set(ARENA.getGamePiecesArrayByType("Fuel"));

        double armEndX = getIntakeArmEndX();
        intakePivot.set(getIntakePivotPose());
        hopper.set(getHopperPose(armEndX));
        intakeRollers.set(getIntakeRollerPose(armEndX));
        shooter.set(getShooterPose());
    }
}