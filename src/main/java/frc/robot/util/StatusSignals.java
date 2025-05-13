package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;

public class StatusSignals {

    private static List<StatusSignal<?>> statusSignals = new ArrayList<>();
    private static StatusSignal<?>[] statusSignalsArray = null;

    public static <T> StatusSignal<T> trackSignal(StatusSignal<T> statusSignal) {
        statusSignals.add(statusSignal);
        return statusSignal;
    }

    public static void trackSignals(StatusSignal<?>... statusSignals) {
        for (StatusSignal<?> statusSignal : statusSignals) {
            trackSignal(statusSignal);
        }
    }

    /**
     * Calls {@link BaseStatusSignal#refreshAll()} on all tracked signals.
     * @return The StatusCode returned by {@link BaseStatusSignal#refreshAll()}.
     */
    public static StatusCode refreshAll() {
        return BaseStatusSignal.refreshAll(getArray());
    }

    private static StatusSignal<?>[] getArray() {
        if (statusSignalsArray == null || statusSignalsArray.length != statusSignals.size()) {
            statusSignalsArray = statusSignals.toArray(new StatusSignal<?>[0]);
        }
        return statusSignalsArray;
    }
}
