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

    public ConveyorLine2 ABC;
    public ConveyorLine2 ABD5;
    public ConveyorLine2 E1;
    public ConveyorLine2 E2;
    public ConveyorLine2 E3;
    public ConveyorLine2 E4;
    public Robot6DOF2 R3;
    public Pusher P2;
    public Pusher P3;

    private ISensorProvider s1abcSens;
    private ISensorProvider s1abd5Sens;
    private ISensorProvider s1e1Sens;
    private ISensorProvider s1e2Sens;
    private ISensorProvider s2e2Sens;
    private ISensorProvider s1e4Sens;
    private ISensorProvider s2e4Sens;

    private IConveyorCommands abcCmd;
    private IConveyorCommands abd5Cmd;
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

    private void Sensor2(SensorCatch sc) {
//      // If there are boxes on1 allora ferma il conveyor 
        if (boxOn1 != null) {
            boxOn1 = sc.box;
        }
        schedule.startSerial();
        {
            // immediately stop conveyor
            e1Cmd.speedSet(0);
            doWelding();
        }
        schedule.end();
    }

    private void Sensor4(SensorCatch sc) {
//      // If there are boxes on1 allora ferma il conveyor 
        if (boxOn2 != null) {
            boxOn1 = sc.box;
        }
        schedule.startSerial();
        {
            // immediately stop conveyor
            e4Cmd.speedSet(0);
            doQualityCheck1();
        }
        schedule.end();
    }

    private void doWelding() {
        // Welding can be performed only if the box is present
        if (boxOn1 != null) {
            // execute the series of assembly operations
            schedule.startSerial();
            {

                r3Cmd.move(driver.getFrameTransform("root.Welding"), 2000);

                schedule.waitTime(15000);
            }
            schedule.end();
            // forget the operated boxes
            boxOn1 = null;

        }
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
        s1e2Sens = E1.createSensors(module);
        s1e2Sens = E2.createSensors(module);
        s2e2Sens = E2.createSensors(module);
        s1e4Sens = E4.createSensors(module);
        s2e4Sens = E4.createSensors(module);

        r3Cmd = R3.create(module);

      // s1e1Sens.registerOnSensors(this::Sensor1, "S1E1");
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
