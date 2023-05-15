/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package my.company;

import com.ttsnetwork.modules.standard.ConveyorLine2;
import com.ttsnetwork.modules.standard.IConveyorCommands;
import com.ttsnetwork.modules.standard.IRobotCommands;
import com.ttsnetwork.modules.standard.ISensorProvider;
import com.ttsnetwork.modules.standard.ProgrammableLogics;
import com.ttsnetwork.modules.standard.Pusher;
import com.ttsnetwork.modules.standard.Robot6DOF2;
import com.ttsnetwork.modulespack.conveyors.ConveyorBox;
import com.ttsnetwork.modulespack.conveyors.SensorCatch;
import org.apache.commons.math3.distribution.RealDistribution;

/**
 *
 * @author jovista
 */
public class PLCL3 extends ProgrammableLogics {

    private ConveyorBox abd6Box;

    public ConveyorLine2 ABC;
    public ConveyorLine2 ABD6;
    public ConveyorLine2 E1;
    public ConveyorLine2 E2;
    public ConveyorLine2 E3;
    public ConveyorLine2 E4;
    public Robot6DOF2 R3;
    public Pusher P2;
    public Pusher P3;

    private ISensorProvider s1abd6Sens;
    private ISensorProvider s2abd6Sens;
    private ISensorProvider s1e1Sens;
    private ISensorProvider s2e1Sens;
    private ISensorProvider s1e2Sens;
    private ISensorProvider s2e2Sens;
    private ISensorProvider s1e4Sens;
    private ISensorProvider s2e4Sens;

    private IConveyorCommands abcCmd;
    private IConveyorCommands abd6Cmd;
    private IConveyorCommands e1Cmd;
    private IConveyorCommands e2Cmd;
    private IConveyorCommands e3Cmd;
    private IConveyorCommands e4Cmd;

    private IRobotCommands r3Cmd;

    private ConveyorBox boxOn1;
    private ConveyorBox boxOn2;
    private ConveyorBox boxOn3;
    private ConveyorBox boxOn4;

    private RealDistribution distribution;

    public boolean busy = false;

    public enum Priority {

        ABC,
        ABD,
        NONE
    }
    public Priority priority = Priority.NONE;
    public boolean abcDetected = false;
    public boolean abdDetected = false;

    private void abd6BoxArrived(SensorCatch sc) {
        abd6Box = sc.box;
        schedule.startSerial();
        {
            abd6Cmd.remove(abd6Box);
            e1Cmd.insert(abd6Box);
        }
        schedule.end();
    }

    private void abd6BoxDetected(SensorCatch sc) {
        schedule.startSerial();
        {
            abdDetected = true;
            if (busy) {
                if (priority.equals(Priority.NONE)) {
                    priority = Priority.ABD;
                }
//            abd6Cmd.speedSet(0);
            } else if (!busy) {
                busy = true;
                abd6Cmd.speedSet(1);
                if (abcDetected) {
                    e1Cmd.speedSet(0);
                }
                abdDetected = false;
            }
        }
        schedule.end();
    }

    private void e1BoxDetected(SensorCatch sc) {
        schedule.startSerial();
        {
            abcDetected = true;
            if (busy) {
                if (priority.equals(Priority.NONE)) {
                    priority = Priority.ABC;
                }
//            e1Cmd.speedSet(0);
            } else if (!busy) {
                busy = true;
                e1Cmd.speedSet(1);
                if (abdDetected) {
                    abd6Cmd.speedSet(0);
                }
                abcDetected = false;
            }
        }

        schedule.end();
    }

    private void e1BoxMerging(SensorCatch sc) {
        schedule.startSerial();
        {
//            abd6Cmd.speedSet(0);
            e1Cmd.speedSet(1);
        }
        schedule.end();
    }

    private void e1BoxStop(SensorCatch sc) {
        schedule.startSerial();
        {
            busy = false;
//            if(priority.equals(Priority.ABC)) {
//                abd6Cmd.speedSet(0);
//                e1Cmd.speedSet(1);
//            } else if(priority.equals(Priority.ABD)) {
//                e1Cmd.speedSet(0);
//                abd6Cmd.speedSet(1);
//            }
            
//            if (!abcDetected) {
//                e1Cmd.speedSet(1);
//            }
//            if (!abdDetected) {
//                abd6Cmd.speedSet(1);
//            }
        }
        schedule.end();
    }

    private void doThermalTreatment() {
        schedule.startSerial();
        {

        }
        schedule.end();
    }

    private void doQualityCheck1() {
        // Welding can be performed only if the box is present
        if (boxOn2 != null) {
            // execute the series of quality check operations
            schedule.startSerial();
            {

                schedule.waitTime(30000);
            }
            schedule.end();
            // forget the operated boxes
            boxOn2 = null;
        }
    }

    private void doQualityCheck2() {
        // Welding can be performed only if the box is present
        if (boxOn4 != null) {
            // execute the series of quality check operations
            schedule.startSerial();
            {

                schedule.waitTime(30000);
            }
            schedule.end();
            // forget the operated boxes
            boxOn4 = null;
        }
    }

    @Override
    public void onInit() {

        e1Cmd = E1.createCommands(module);
        e2Cmd = E2.createCommands(module);
        e3Cmd = E3.createCommands(module);
        e4Cmd = E4.createCommands(module);

        s1e1Sens = E1.createSensors(module);
        s2e1Sens = E1.createSensors(module);
        s1e2Sens = E1.createSensors(module);
        s2e2Sens = E2.createSensors(module);
        s1e4Sens = E4.createSensors(module);
        s2e4Sens = E4.createSensors(module);

        r3Cmd = R3.create(module);

        abd6Cmd = ABD6.createCommands(module);
        s1abd6Sens = ABD6.createSensors(module);
        s2abd6Sens = ABD6.createSensors(module);

        s1abd6Sens.registerOnSensors(this::abd6BoxDetected, "S1ABD6");
        s2abd6Sens.registerOnSensors(this::abd6BoxArrived, "S2ABD6");
        s1e1Sens.registerOnSensors(this::e1BoxDetected, "S1E1");
        s2e1Sens.registerOnSensors(this::e1BoxMerging, "S2E1");
        s1e2Sens.registerOnSensors(this::e1BoxStop, "S1E2");

        schedule.startSerial();
        {
            schedule.waitTime(1000);
//            srcCmd.create(null);
//            schedule.waitTime(4000);
//            schedule.callFunction(this::extractP2);
//            schedule.callFunction(this::extractP3);
//            schedule.waitTime(1000);
//            schedule.callFunction(this::retractP2);
//            schedule.callFunction(this::retractP3);
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
