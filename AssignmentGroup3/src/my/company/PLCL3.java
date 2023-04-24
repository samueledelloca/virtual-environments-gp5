/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package my.company;

import com.ttsnetwork.modules.standard.ProgrammableLogics;
import com.ttsnetwork.modules.standard.Pusher;

/**
 *
 * @author samu
 */
public class PLCL3 extends ProgrammableLogics {

    public Pusher P2;
    public Pusher P3;

    @Override
    public void onInit() {
        schedule.startSerial();
        {
            schedule.waitTime(1000);
//            srcCmd.create(null);
//            schedule.waitTime(4000);
            schedule.callFunction(this::extractP2);
            schedule.callFunction(this::extractP3);
            schedule.waitTime(1000);
            schedule.callFunction(this::retractP2);
            schedule.callFunction(this::retractP3);
        }
        schedule.end();
    }

    public void extractP2() {
        P2.cmdStable.write(true);
    }

    public void retractP2() {
        P2.cmdStable.write(false);
    }

    public void extractP3() {
        P3.cmdStable.write(true);
    }

    public void retractP3() {
        P3.cmdStable.write(false);
    }
}
