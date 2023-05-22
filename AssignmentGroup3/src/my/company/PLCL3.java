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
import org.apache.commons.math3.distribution.UniformRealDistribution;

/**
 *
 * @author jovista
 */
public class PLCL3 extends ProgrammableLogics {

    private ConveyorBox abd7Box;

    public ConveyorLine2 ABC;
    public ConveyorLine2 ABD6;
    public ConveyorLine2 ABD7;
    public ConveyorLine2 E1;
    public ConveyorLine2 E2;
    public ConveyorLine2 E3;
    public ConveyorLine2 E4;
    public ConveyorLine2 E5;
    public ConveyorLine2 E6;
    public Robot6DOF2 R3;
    public Pusher P2;
    public Pusher P3;

    private ISensorProvider s1abd6Sens;
    private ISensorProvider s1abd7Sens;
    private ISensorProvider s1e1Sens;
    private ISensorProvider s2e1Sens;
    private ISensorProvider s1e2Sens;
    private ISensorProvider s1e3Sens;
    private ISensorProvider s1e4Sens;
    private ISensorProvider s1e6Sens;
    private ISensorProvider s2e6Sens;

    private IConveyorCommands abcCmd;
    private IConveyorCommands abd6Cmd;
    private IConveyorCommands abd7Cmd;
    private IConveyorCommands e1Cmd;
    private IConveyorCommands e2Cmd;
    private IConveyorCommands e3Cmd;
    private IConveyorCommands e4Cmd;
    private IConveyorCommands e5Cmd;
    private IConveyorCommands e6Cmd;

    private IRobotCommands r3Cmd;

    private ConveyorBox boxOn1;
    private ConveyorBox boxOn2;
    private ConveyorBox boxOn3;
    private ConveyorBox boxOn4;

    private UniformRealDistribution distribution;

    public boolean busy = false;
    public boolean abdDetected = false;

    private void abd7BoxArrived(SensorCatch sc) {
        abd7Box = sc.box;
        schedule.startSerial();
        {
            abd7Cmd.remove(abd7Box);
            e2Cmd.insert(abd7Box);
        }
        schedule.end();
    }

    private void abd6BoxDetected(SensorCatch sc) {
        schedule.startSerial();
        {
            abdDetected = true;
            if (busy) {
                abd7Cmd.speedSet(0);
            } else {
                busy = true;
                abd7Cmd.speedSet(1);
                e1Cmd.speedSet(0);
                abdDetected = false;
            }
        }
        schedule.end();
    }

    private void e1BoxDetected(SensorCatch sc) {
        schedule.startSerial();
        {
            if (!busy) {
                busy = true;
                e1Cmd.speedSet(1);
                abd7Cmd.speedSet(0);
            }
        }
        schedule.end();
    }

    private void eBoxMerging(SensorCatch sc) {
        schedule.startSerial();
        {

        }
        schedule.end();
    }

    private void eBoxExiting(SensorCatch sc) {
        schedule.startSerial();
        {
            busy = false;
            if (abdDetected) {
                abd7Cmd.speedSet(1);
            } else {
                abd7Cmd.speedSet(0);
                e1Cmd.speedSet(1);
            }
        }
        schedule.end();
    }

    private void eBoxFirstStop(SensorCatch sc) {
        schedule.startSerial();
        {
            e3Cmd.speedSet(0);
            schedule.waitTime(500);
            if (distribution.sample() >= 0.7) {
                sc.box.entity.setProperty("defective", true);
            } else {
                sc.box.entity.setProperty("defective", false);
            }
            e3Cmd.speedSet(1);
        }
        schedule.end();
    }
    
    private void eBoxSecondStop(SensorCatch sc) {
        schedule.startSerial();
        {
            e5Cmd.speedSet(0);
            schedule.waitTime(500);
            if (distribution.sample() >= 0.8) {
                sc.box.entity.setProperty("defective", true);
            } else {
                sc.box.entity.setProperty("defective", false);
            }
            e5Cmd.speedSet(1);
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

    private void pushWithP2(SensorCatch sc) {
        if (sc.box.entity.getProperty("defective").equals(true)) {
            schedule.startSerial();
            {
                e4Cmd.speedSet(0);
                schedule.callFunction(this::extractP2);
                schedule.waitTime(1000);
                schedule.callFunction(this::retractP2);
                e4Cmd.speedSet(1);
            }
            schedule.end();
        }
    }

    private void pushWithP3(SensorCatch sc) {
        if (sc.box.entity.getProperty("defective").equals(true)) {
            schedule.startSerial();
            {
                e6Cmd.speedSet(0);
                schedule.callFunction(this::extractP3);
                schedule.waitTime(1000);
                schedule.callFunction(this::retractP3);
                e6Cmd.speedSet(1);
            }
            schedule.end();
        }
    }

    @Override
    public void onInit() {

        distribution = model.getRandomGenerator().getUniformRealDistribution(0, 1);

        e1Cmd = E1.createCommands(module);
        e2Cmd = E2.createCommands(module);
        e3Cmd = E3.createCommands(module);
        e4Cmd = E4.createCommands(module);
        e5Cmd = E5.createCommands(module);
        e6Cmd = E6.createCommands(module);

        s1e1Sens = E1.createSensors(module);
        s2e1Sens = E1.createSensors(module);
        s1e2Sens = E2.createSensors(module);
        s1e3Sens = E3.createSensors(module);
        s1e4Sens = E4.createSensors(module);
        s1e6Sens = E6.createSensors(module);
        s2e6Sens = E6.createSensors(module);

        r3Cmd = R3.create(module);

        abd6Cmd = ABD6.createCommands(module);
        s1abd6Sens = ABD6.createSensors(module);

        abd7Cmd = ABD7.createCommands(module);
        s1abd7Sens = ABD7.createSensors(module);

        s1abd6Sens.registerOnSensors(this::abd6BoxDetected, "S1ABD6");
        s1abd7Sens.registerOnSensors(this::abd7BoxArrived, "S1ABD7");
        s1e1Sens.registerOnSensors(this::e1BoxDetected, "S1E1");
        s2e1Sens.registerOnSensors(this::eBoxMerging, "S2E1");
        s1e2Sens.registerOnSensors(this::eBoxExiting, "S1E2");
        s1e3Sens.registerOnSensors(this::eBoxFirstStop, "S1E3");
        s1e4Sens.registerOnSensors(this::pushWithP2, "S1E4");
        s1e6Sens.registerOnSensors(this::eBoxSecondStop, "S1E6");
        s2e6Sens.registerOnSensors(this::pushWithP3, "S2E6");

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
