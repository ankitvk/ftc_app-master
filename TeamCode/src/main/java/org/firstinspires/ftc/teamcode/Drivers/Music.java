package org.firstinspires.ftc.teamcode.Drivers;

import android.content.Context;
import android.media.MediaPlayer;

import com.qualcomm.ftcrobotcontroller.R;

import java.io.IOException;
import java.util.Random;

public class Music {
    //The player handling the audio
    private static MediaPlayer mediaPlayer = null;
    private Context context;
    //Start the wubs]

    public Music(Context context){
        this.context = context;
    }

    private int songChoice(Songs song){
        switch (song){
            case SEA_SHANTY_2:
                return R.raw.sea_shanty_2;
            case ALMA_MATER:
                return R.raw.alma_mater;
            case SICKO_MODE:
            case MO_BAMBA:
            case CRAB_RAVE:
            case COUNTRY_ROADS:
            case OBAMA_IS_GONE:
            default:
                return R.raw.errormessage;
        }
    }

    private int songChoice(){
        Random rand = new Random();
        int song = rand.nextInt(Songs.values().length);
        switch (song){
            case 0:
                return R.raw.sea_shanty_2;
            case 1:
                return R.raw.alma_mater;
            default:
                return R.raw.errormessage;
        }
    }
    public void start(Songs song) {
        if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(context,songChoice(song) );
        mediaPlayer.seekTo(0);
        mediaPlayer.start();
    }

    public void start(){
        if (mediaPlayer == null) mediaPlayer = MediaPlayer.create(context,songChoice());
        mediaPlayer.seekTo(0);
        mediaPlayer.start();
    }
    //Stop the wubs
    public void stop() {
        if (mediaPlayer != null) {
            mediaPlayer.stop();
            try { mediaPlayer.prepare(); }
            catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public enum Songs{
        SEA_SHANTY_2,
        ALMA_MATER,
        SICKO_MODE,
        MO_BAMBA,
        CRAB_RAVE,
        COUNTRY_ROADS,
        OBAMA_IS_GONE

    }
}
