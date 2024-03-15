package org.firstinspires.ftc.teamcode.log;

import com.acmerobotics.dashboard.config.Config;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.internal.bind.TypeAdapters;
import com.google.gson.stream.JsonWriter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.poser.Angle;
import org.firstinspires.ftc.teamcode.poser.Distance;
import org.firstinspires.ftc.teamcode.poser.Distance2;
import org.firstinspires.ftc.teamcode.poser.Pose;
import org.firstinspires.ftc.teamcode.poser.Vector2;

import java.io.FileWriter;
import java.io.IOException;
import java.lang.reflect.Array;
import java.util.List;

@Config
public class Log {
    public static boolean ENABLED = true;

    private Telemetry telemetry;
    private FileWriter logFile;

    public Log(OpMode opMode) {
        if (!ENABLED) return;

        this.telemetry = opMode.telemetry;

        String now = "" + System.currentTimeMillis() / 1000; // i give up.
        String fileName = "log-" + opMode.getClass().getSimpleName() + "-" + now + ".log";
        try {
            this.logFile = new FileWriter(AppUtil.getInstance().getSettingsFile(fileName));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    public Plank chop(String origin) {
        return new Plank(this, origin);
    }

    public void addLine(String origin, String line) {
        if (!ENABLED) return;

//        System.out.format("[%s] %s\n", origin, line);
        RobotLog.ii(origin, line);
        if (telemetry != null) telemetry.addLine(line);
        if (logFile != null) {
            JsonObject obj = new JsonObject();
            obj.addProperty("origin", origin);
            obj.addProperty("timestamp", System.currentTimeMillis());
            obj.addProperty("kind", "line");
            obj.addProperty("line", line);
            logLine(obj);
        }
    }

    public void addLine(String origin, String format, Object... args) {
        this.addLine(origin, String.format(format, args));
    }

    public void addData(String origin, String key, Object value) {
        if (!ENABLED) return;

//        System.out.format("[%s] %s : %s\n", origin, key, value);
        RobotLog.ii(origin, "%s : %s", key, value);
        if (telemetry != null) telemetry.addData(key, value);
        if (logFile != null) {
            JsonObject obj = new JsonObject();
            obj.addProperty("origin", origin);
            obj.addProperty("timestamp", System.currentTimeMillis());
            obj.addProperty("kind", "data");
            obj.addProperty("key", key);
            obj.add("value", objToJson(value));
            logLine(obj);
        }
    }

    private void logLine(JsonElement obj) {
        try {
            TypeAdapters.JSON_ELEMENT.write(new JsonWriter(logFile), obj);
            logFile.write('\n');
            logFile.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private static JsonElement objToJson(Object v) {
        if (v == null) {
            return JsonNull.INSTANCE;
        } else if (v instanceof String) {
            return new JsonPrimitive((String) v);
        } else if (v instanceof Number) {
            return new JsonPrimitive((Number) v);
        } else if (v instanceof Boolean) {
            return new JsonPrimitive((Boolean) v);
        } else if (v instanceof List) {
            JsonArray arr = new JsonArray();
            for (Object el : (List<?>)v) {
                arr.add(objToJson(el));
            }
            return arr;
        } else if (v.getClass().isArray()) {
            JsonArray arr = new JsonArray();
            int length = Array.getLength(v);
            for (int i = 0; i < length; i ++) {
                arr.add(objToJson(Array.get(v, i)));
            }
            return arr;
        } else if (v instanceof Distance) {
            JsonObject obj = new JsonObject();
            obj.addProperty("kind", "Distance");
            obj.addProperty("val", ((Distance) v).valInMM());
            return obj;
        } else if (v instanceof Angle) {
            JsonObject obj = new JsonObject();
            obj.addProperty("kind", "Angle");
            obj.addProperty("val", ((Angle) v).valInRadians());
            return obj;
        } else if (v instanceof Distance2) {
            JsonObject obj = new JsonObject();
            obj.addProperty("kind", "Distance2");
            obj.addProperty("x", ((Distance2) v).x.valInMM());
            obj.addProperty("y", ((Distance2) v).y.valInMM());
            return obj;
        } else if (v instanceof Vector2) {
            JsonObject obj = new JsonObject();
            obj.addProperty("kind", "Vector2");
            obj.addProperty("x", ((Vector2) v).x);
            obj.addProperty("y", ((Vector2) v).y);
            return obj;
        } else if (v instanceof Pose) {
            JsonObject obj = new JsonObject();
            obj.addProperty("kind", "Pose");
            obj.addProperty("x", ((Pose) v).pos.x.valInMM());
            obj.addProperty("y", ((Pose) v).pos.y.valInMM());
            obj.addProperty("theta", ((Pose) v).yaw.valInRadians());
            return obj;
        } else {
            return new JsonPrimitive(v.toString());
        }
    }
}
