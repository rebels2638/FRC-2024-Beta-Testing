package frc.robot.commands.audio;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.audio.AudioPlayer;

public class playMusic extends Command {
    private AudioPlayer audioPlayerSubsystem;
    private int toggle;
    private int maxBound;

    public playMusic(AudioPlayer audioplayerSubsystem) {
        this.audioPlayerSubsystem = audioplayerSubsystem;
        this.toggle = 0;
        this.maxBound = 2; // exclusive, also maxBound is silent
        addRequirements(audioplayerSubsystem);
    }
    
    @Override
    public void execute() {
        System.err.println(this.toggle);
        if (this.toggle == this.maxBound) {
            this.audioPlayerSubsystem.stop();
            this.toggle = 0;
        }
        else {
            this.audioPlayerSubsystem.play(this.toggle);
            this.toggle++;
        }

        // this.audioPlayerSubsystem.play(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
