package com.stuypulse.robot.commands.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.stuypulse.robot.subsystems.swerve.CommandSwerveDrivetrain;

import edu.wpi.first.wpilibj2.command.Command;

public class SwerveXMode extends Command {
    private CommandSwerveDrivetrain swerve;

    public SwerveXMode() {
        swerve = CommandSwerveDrivetrain.getInstance();
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        SwerveRequest request = new SwerveRequest.SwerveDriveBrake();
        CommandSwerveDrivetrain.getInstance().setControl(request);
    }
}