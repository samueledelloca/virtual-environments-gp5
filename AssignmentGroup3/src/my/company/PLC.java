/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package my.company;

import com.ttsnetwork.modules.standard.ISource;
import com.ttsnetwork.modules.standard.ParametricSource2;
import com.ttsnetwork.modules.standard.ProgrammableLogics;
import com.ttsnetwork.modules.standard.Pusher;

/**
 *
 * @author samu
 */
public class PLC extends ProgrammableLogics {

//    public ParametricSource2 SA;
//    ISource srcCmd;

    public Pusher P1;

    @Override
    public void onInit() {
//        srcCmd = SA.createSkill(module);

        schedule.startSerial();
        {
            schedule.waitTime(1000);
//            srcCmd.create(null);
//            schedule.waitTime(4000);
//            schedule.callFunction(this::extract);
//            schedule.waitTime(1000);
//            schedule.callFunction(this::retract);
        }
        schedule.end();
    }

    public void extract() {
        P1.cmdStable.write(true);
    }

    public void retract() {
        P1.cmdStable.write(false);
    }

//    schedule.startSerial ();
//
//    {
//        c1Cmd.speedSet(0);
//        schedule.callFunction(this::extract);
//        schedule.waitTime(1000);
//        schedule.callFunction(this::retract);
//        c1Cmd.speedSet(1);
//    }
//
//    schedule.end ();
}
