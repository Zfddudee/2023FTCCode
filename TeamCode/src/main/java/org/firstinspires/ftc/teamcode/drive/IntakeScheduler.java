package org.firstinspires.ftc.teamcode.drive;

import java.util.ArrayList;
import java.util.Date;

public final class IntakeScheduler {
    private ArrayList<ScheduledRunnable> storage = new ArrayList<>();
    private Thread thread;

    public void start() {
        thread = new Thread(() -> {
            while(!thread.isInterrupted()) {
                update();
            }
        });

        thread.start();
    }

    public void stop() {
        if(thread == null) return;

        thread.interrupt();
    }

    private void schedule(ScheduledRunnable runnable) {
        storage.add(runnable);
    }

    public void schedule(Runnable runnable, long runAfter) {
        schedule(new ScheduledRunnable(runAfter + getLatestTime()) {
            @Override
            public void run() {
                runnable.run();
            }
        });
    }

    public void update() {
        if(storage.size() == 0) return;

        ScheduledRunnable current = storage.get(0);

        if(current.runAt <= getTime()) {
            storage.remove(0);
            current.run();
        }
    }

    private long getTime() {
        return new Date().getTime();
    }

    private long getLatestTime() {
        if(storage.size() == 0) return 0;
        return storage.get(storage.size() - 1).runAt - getTime();
    }

    private abstract class ScheduledRunnable implements Runnable {
        public final long runAt;
        public ScheduledRunnable(long runAfter) {
            this.runAt = getTime() + runAfter;
        }

        public abstract void run();
    }
}