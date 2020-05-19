package frc.robot.commands.claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class ClawCloseCommand extends CommandBase {

    private Claw claw;

    public ClawCloseCommand(Claw claw) {
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.close();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}