package frc.robot.subsystems.audio;

import java.util.ArrayList;
import java.util.Arrays

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AudioPlayer extends SubsystemBase {
    
    private Orchestra orchestra;
    ArrayList<TalonFX> motors;

    public AudioPlayer() {
        ArrayList<TalonFX> motors = new ArrayList<TalonFX> (
            Arrays.asList(new TalonFX(0), new Falcon(1), new TalonFX(2), new TalonFX(3), 
                new TalonFX(4), new TalonFX(5), new TalonFX(6), new TalonFX(7)));

        orchestra = new Orchestra(motors);
    }

    public void play(int songID) {
        orchestra.loadMusic("src/main/java/frc/robot/subsystems/audio/tracks/"+Integer.tostring(songID)+".chrp");
    }

    public void stop() {
        orchestra.stop()
    }

}
