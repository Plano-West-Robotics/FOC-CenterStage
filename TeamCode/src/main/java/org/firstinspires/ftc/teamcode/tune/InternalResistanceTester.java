package org.firstinspires.ftc.teamcode.tune;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.log.Plank;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.ArrayList;

@TeleOp
public class InternalResistanceTester extends LinearOpMode {
    Hardware hardware;
    LynxModule eh;
    Plank log;

    @Override
    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(this);
        eh = hardwareMap.get(LynxModule.class, "Expansion Hub 3");
        log = hardware.log.chop("InternalResistance");
        Drive drive = new Drive(hardware, 0.7);

        waitForStart();

        ArrayList<VoltageCurrentPair> measurements = new ArrayList<>();
        VoltageCurrentPair measurement;

        log.addLine("No load:");
        do { measurement = tryMeasure(); } while (measurement == null);
        measurements.add(measurement);

        log.addLine("Intake load:");
        hardware.intake.setPower(1);
        hardware.ramp.setPower(1);
        do { measurement = tryMeasure(); } while (measurement == null);
        measurements.add(measurement);

        log.addLine("Intake+lift load:");
        hardware.liftL.setPower(0.5);
        hardware.liftR.setPower(0.5);
        do { measurement = tryMeasure(); } while (measurement == null);
        measurements.add(measurement);
        hardware.liftL.setPower(0);
        hardware.liftR.setPower(0);

        log.addLine("Intake+turn load:");
        drive.driveOld(0, 0, 1);
        do { measurement = tryMeasure(); } while (measurement == null);
        measurements.add(measurement);

        log.addLine("Turn load:");
        hardware.intake.setPower(0);
        hardware.ramp.setPower(0);
        drive.driveOld(0, 0, -1);
        do { measurement = tryMeasure(); } while (measurement == null);
        measurements.add(measurement);

        drive.stop();

        log.addLine("Done collecting data.");
        StringBuilder dataString = new StringBuilder("P=");
        formatHelper2(dataString, measurements);
        log.addLine(dataString.toString());

        double vMean = 0;
        double iMean = 0;
        double prodMean = 0;
        double vSqMean = 0;
        double iSqMean = 0;
        for (VoltageCurrentPair m : measurements) {
            double v = m.voltage;
            double i = m.current;
            vMean += v;
            iMean += i;
            prodMean += v * i;
            vSqMean += v * v;
            iSqMean += i * i;
        }
        int n = measurements.size();
        vMean /= n;
        iMean /= n;
        prodMean /= n;
        vSqMean /= n;
        iSqMean /= n;
        double vStddevSq = vSqMean - vMean * vMean;
        double iStddevSq = iSqMean - iMean * iMean;

        // V0 = I(R_i) + V
        // V0 = I(R_i) + V

        // V = V0 - I(R_i)
        // [ V1 ]   [ 1 -I1 ]
        // [ V2 ] = [ 1 -I2 ] * [ V0  ]
        // [ V3 ]   [ 1 -I3 ]   [ R_i ]


        //                   [ 1 -I1 ]
        // [  1   1   1  ] * [ 1 -I2 ] = [    N   -I_tot  ]
        // [ -I1 -I2 -I3 ]   [ 1 -I3 ] = [ -I_tot I^2_tot ]

        // [    N   -I_tot  ] ^ -1                             [ I^2_tot I_tot ]
        // [ -I_tot I^2_tot ]      = 1/(N*I^2_tot - I_tot^2) * [  I_tot    N   ]

        // [ I^2_tot I_tot ]   [  1   1   1  ]   [ I^2_tot - I_i*I_tot ]
        // [  I_tot    N   ] * [ -I1 -I2 -I3 ] = [   I_tot - N * I_i   ]

        // [ I^2_tot - I_i*I_tot ]       [ total(V_i*I^2_tot - V_i*I_i*I_tot) ]
        // [   I_tot - N * I_i   ] * V = [    total(V_i*I_tot - N*V_i*I_i)    ]

        // V0 = total(V_i*I^2_tot - V_i*I_i*I_tot)/(N*I^2_tot - I_tot^2)
        // R_i = total(V_i*I_tot - N*V_i*I_i)/(N*I^2_tot - I_tot^2)
        // R_i = (V_tot*I_tot - N*(VI)_tot)/(N*I^2_tot - I_tot^2)
        // R_i = (V_tot*I_tot - N*(VI)_tot)/(I_stddev^2 * N^2)
        // R_i = (V_avg*I_avg - (VI)_avg)/(I_stddev^2)
        // V0 = (total(V_i*I^2_tot) - total(V_i*I_i*I_tot))/(N*I^2_tot - I_tot^2)
        // V0 = (total(V_i)*I^2_tot - total(V_i*I_i)*I_tot)/(I_stddev^2 * N^2)
        // V0 = (V_tot*I^2_tot - (VI)_tot*I_tot)/(I_stddev^2 * N^2)
        // V0 = (V_avg*I^2_avg - (VI)_avg*I_avg)/(I_stddev^2)

        // N*I^2_tot - I_tot^2 = (N * stddev)^2
        // stddev = sqrt(I^2_tot / N - I_tot^2 / N^2);
        // stddev^2 = I^2_tot / N - I_tot^2 / N^2;
        // stddev^2 * N^2 = N*I^2_tot - I_tot^2;

        double tmp = vMean * iMean - prodMean;
        double rInternal = tmp / iStddevSq;
        double vNominal = (vMean * iSqMean - prodMean * iMean) / iStddevSq;
        double correlationCoeff = -tmp / Math.sqrt(vStddevSq * iStddevSq);

        log.addData("Internal resistance (ohms)", rInternal);
        log.addData("Nominal voltage (V)", vNominal);
        log.addData("Correlation coefficient", correlationCoeff);
    }

    private void formatHelper(StringBuilder builder, ArrayList<Double> list) {
        builder.append("\\left[");
        boolean first = true;
        for (double v : list) {
            if (first) first = false;
            else builder.append(",");
            builder.append(v);
        }
        builder.append("\\right]");
    }

    private void formatHelper2(StringBuilder builder, ArrayList<VoltageCurrentPair> list) {
        builder.append("\\left[");
        boolean first = true;
        for (VoltageCurrentPair v : list) {
            if (first) first = false;
            else builder.append(",");
            builder.append("\\left(");
            builder.append(v.current);
            builder.append(",");
            builder.append(v.voltage);
            builder.append("\\right)");
        }
        builder.append("\\right]");
    }

    public VoltageCurrentPair tryMeasure() {
        long stopTime = System.nanoTime() + 1 * 1000 * 1000 * 1000;
        ArrayList<Double> voltages = new ArrayList<>();
        ArrayList<Double> currents = new ArrayList<>();
        while (System.nanoTime() < stopTime) {
            voltages.add(eh.getInputVoltage(VoltageUnit.VOLTS));
            currents.add(eh.getCurrent(CurrentUnit.AMPS));
        }

        double vStddev = stddev(voltages);
        double iStddev = stddev(currents);
        double vMean = mean(voltages);
        double iMean = mean(currents);

        log.addLine("Measurement:");

        StringBuilder voltagesString = new StringBuilder("    V=");
        formatHelper(voltagesString, voltages);
        log.addLine(voltagesString.toString());

        StringBuilder currentsString = new StringBuilder("    I=");
        formatHelper(currentsString, currents);
        log.addLine(currentsString.toString());

        log.addLine("    Deviations: ±%f V, ±%f A", vStddev, iStddev);
        log.addLine("    Means: %f V, %f A", vMean, iMean);

        if (vStddev > 0.05 || iStddev > 0.12) return null;
        return new VoltageCurrentPair(mean(voltages), mean(currents));
    }

    static class VoltageCurrentPair {
        double voltage;
        double current;

        public VoltageCurrentPair(double voltage, double current) {
            this.voltage = voltage;
            this.current = current;
        }
    }

    double mean(Iterable<Double> arr) {
        double total = 0;
        int count = 0;
        for (double v : arr) {
            total += v;
            count += 1;
        }
        return total / count;
    }

    double stddev(Iterable<Double> arr) {
        double total = 0;
        double total_squares = 0;
        int count = 0;
        for (double v : arr) {
            total += v;
            total_squares += v * v;
            count += 1;
        }
        return Math.sqrt(total_squares / count - Math.pow(total / count, 2));
    }
}
