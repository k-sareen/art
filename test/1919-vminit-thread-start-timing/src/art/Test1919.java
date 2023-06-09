/*
 * Copyright (C) 2011 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package art;

public class Test1919 {
    public static final boolean PRINT_ALL_THREADS = false;

    public static void run() throws Exception {
        Thread testing_thread = getTestingThread();
        // TODO(b/124284724): For unknown reasons the testing thread will sometimes SEGV after the
        // test has otherwise completed successfully. This has only been observed on the release
        // version of art (libart.so) and I haven't had any luck reproing it. I assume it has
        // something to do with racing between the DetachCurrentThread and shutdown but I'm not
        // sure. Since the runtime normally never shuts down anyway for now I'll just ensure
        // everything gets cleaned up early to prevent the problem from showing up.
        testing_thread.join();
        for (Event e : getEvents()) {
            if (e.thr != null) {
                if (PRINT_ALL_THREADS ||
                        e.thr.equals(Thread.currentThread()) ||
                        e.thr.equals(testing_thread)) {
                    System.out.println(e.name + ": " + e.thr.getName());
                }
            }
        }
    }

    static class Event {
        public final String name;
        public final Thread thr;
        public Event(String name, Thread thr) {
            this.name = name;
            this.thr = thr;
        }
    }

    public static Event[] getEvents() {
        String[] ns = getEventNames();
        Thread[] ts = getEventThreads();
        Event[] es = new Event[Math.min(ns.length, ts.length)];
        for (int i = 0; i < es.length; i++) {
            es[i] = new Event(ns[i], ts[i]);
        }
        return es;
    }

    public static native String[] getEventNames();
    public static native Thread[] getEventThreads();

    public static native Thread getTestingThread();
}
