package com.stuypulse.robot.util;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.math.geometry.Pose3d;

import com.stuypulse.robot.constants.Field;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static edu.wpi.first.units.Units.Inches;


public class Simulation {
    public static final Arena2026Rebuilt ARENA = new Arena2026Rebuilt(true);
    public static void configure() {
        SimulatedArena.overrideInstance(ARENA);
    }
    
    final DriveTrainSimulationConfig driveTrainSimulationConfig = DriveTrainSimulationConfig.Default()
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(COTS.ofMark4(
            DCMotor.getKrakenX60(1), // Drive motor is a Kraken X60
            DCMotor.getKrakenX60(1), // Steer motor is a Kraken X60
            COTS.WHEELS.COLSONS.cof, // Use the COF for Colson Wheels
            3)) // L3 Gear ratio
        // Configures the track length and track width (spacing between swerve modules)
        .withTrackLengthTrackWidth(Inches.of(24), Inches.of(24))
        // Configures the bumper size (dimensions of the robot bumper)
        .withBumperSize(Inches.of(30), Inches.of(30));


    /* Create a swerve drive simulation */
    this.swerveDriveSimulation = new SwerveDriveSimulation(
        // Specify Configuration
        driveTrainSimulationConfig,
        // Specify starting pose
        new Pose2d(3, 3, new Rotation2d())
    );
    SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
}

    public interface SwerveDrive extends Subsystem {







    // //idk where this is supposed to go, please move
    // RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
    //     robotSimulationWorldPose.getTranslation(),
    //     // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
    //     new Translation2d(0.2, 0),
    //     // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
    //     chassisSpeedsFieldRelative,
    //     // The shooter facing direction is the same as the robot’s facing direction
    //     robotSimulationWorldPose.getRotation()
    //             // Add the shooter's rotation
    //             + shooterRotation,
    //     // Initial height of the flying note
    //     0.45,
    //     // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
    //     velocityRPM / 6000 * 20,
    //     // The angle at which the note is launched
    //     Math.toRadians(55)
    // );