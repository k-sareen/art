/*
 * Copyright (C) 2016 The Android Open Source Project
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

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;

class SampleObject {
    public static boolean sHashCodeInvoked = false;
    private int i;

    public SampleObject(int i) {
        this.i = i;
    }

    public boolean equals(Object obj) {
        return (obj instanceof SampleObject) && (i == ((SampleObject)obj).i);
    }

    public int hashCode() {
        sHashCodeInvoked = true;
        Main.assertIsManaged();
        Main.deoptimizeAll();
        Main.assertIsInterpreted();
        return i % 64;
    }
}

public class Main {
    static boolean sFlag = false;

    public static native void deoptimizeAll();
    public static native void undeoptimizeAll();
    public static native void assertIsInterpreted();
    public static native void assertIsManaged();
    public static native void assertCallerIsInterpreted();
    public static native void disableStackFrameAsserts();
    public static native boolean hasJit();
    private static native void ensureJitCompiled(Class<?> itf, String method_name);

    public static void execute(Runnable runnable) throws Exception {
      Thread t = new Thread(runnable);
      t.start();
      t.join();
    }

    public static void ensureAllJitCompiled() {
        ensureJitCompiled(HashMap.class, "hash");
        ensureJitCompiled(Main.class, "$noinline$run1");
        ensureJitCompiled(Main.class, "$noinline$run2");
        ensureJitCompiled(Main.class, "$noinline$run3A");
        ensureJitCompiled(Main.class, "$noinline$run3B");
        ensureJitCompiled(SampleObject.class, "hashCode");
    }

    public static void main(String[] args) throws Exception {
        System.loadLibrary(args[0]);
        // Only test stack frames in compiled mode.
        if (!hasJit()) {
          disableStackFrameAsserts();
        }

        // Just declare a new int array so that the int arrays are resolved properly when JITing.
        int[] tmp = new int[3];
        ensureAllJitCompiled();

        final HashMap<SampleObject, Long> map = new HashMap<SampleObject, Long>();

        // Single-frame deoptimization that covers partial fragment.
        execute(new Runnable() {
            public void run() {
                ensureJitCompiled(this.getClass(), "runInternal");
                runInternal();
            }

            public void runInternal() {
                int[] arr = new int[3];
                assertIsManaged();
                int res = $noinline$run1(arr);
                assertIsManaged();  // Only single frame is deoptimized.
                if (res != 79) {
                    System.out.println("Failure 1!");
                    System.exit(0);
                }
            }
        });

        // Single-frame deoptimization that covers a full fragment.
        execute(new Runnable() {
            public void run() {
                ensureJitCompiled(this.getClass(), "runInternal");
                runInternal();
            }

            public void runInternal() {
                try {
                    int[] arr = new int[3];
                    assertIsManaged();
                    // Use reflection to call $noinline$run2 so that it does
                    // full-fragment deoptimization since that is an upcall.
                    Class<?> cls = Class.forName("Main");
                    Method method = cls.getDeclaredMethod("$noinline$run2", int[].class);
                    double res = (double)method.invoke(Main.class, arr);
                    assertIsManaged();  // Only single frame is deoptimized.
                    if (res != 79.3d) {
                        System.out.println("Failure 2!");
                        System.exit(0);
                    }
                } catch (Exception e) {
                    e.printStackTrace(System.out);
                }
            }
        });

        // Full-fragment deoptimization.
        execute(new Runnable() {
            public void run() {
                ensureJitCompiled(this.getClass(), "runInternal");
                runInternal();
            }

            public void runInternal() {
                assertIsManaged();
                float res = $noinline$run3B();
                assertIsInterpreted();  // Every deoptimizeable method is deoptimized.
                if (res != 0.034f) {
                    System.out.println("Failure 3!");
                    System.exit(0);
                }
            }
        });

        undeoptimizeAll();  // Make compiled code useable again.
        ensureAllJitCompiled();

        // Partial-fragment deoptimization.
        execute(new Runnable() {
            public void run() {
                ensureJitCompiled(this.getClass(), "runInternal");
                ensureJitCompiled(HashMap.class, "hash");
                runInternal();
            }

            public void runInternal() {
                try {
                    assertIsManaged();
                    map.put(new SampleObject(10), Long.valueOf(100));
                    assertIsInterpreted();  // Every deoptimizeable method is deoptimized.
                } catch (Exception e) {
                    e.printStackTrace(System.out);
                }
            }
        });

        undeoptimizeAll();  // Make compiled code useable again.
        ensureAllJitCompiled();

        if (!SampleObject.sHashCodeInvoked) {
            System.out.println("hashCode() method not invoked!");
        }
        if (map.get(new SampleObject(10)) != 100) {
            System.out.println("Wrong hashmap value!");
        }
        System.out.println("Finishing");
    }

    public static int $noinline$run1(int[] arr) {
        assertIsManaged();
        // Prevent inlining.
        if (sFlag) {
            throw new Error();
        }
        boolean caught = false;
        // BCE will use deoptimization for the code below.
        try {
            arr[0] = 1;
            arr[1] = 1;
            arr[2] = 1;
            // This causes AIOOBE and triggers deoptimization from compiled code.
            arr[3] = 1;
        } catch (ArrayIndexOutOfBoundsException e) {
            assertIsInterpreted(); // Single-frame deoptimization triggered.
            caught = true;
        }
        if (!caught) {
            System.out.println("Expected exception");
        }
        assertIsInterpreted();
        return 79;
    }

    public static double $noinline$run2(int[] arr) {
        assertIsManaged();
        // Prevent inlining.
        if (sFlag) {
            throw new Error();
        }
        boolean caught = false;
        // BCE will use deoptimization for the code below.
        try {
            arr[0] = 1;
            arr[1] = 1;
            arr[2] = 1;
            // This causes AIOOBE and triggers deoptimization from compiled code.
            arr[3] = 1;
        } catch (ArrayIndexOutOfBoundsException e) {
            assertIsInterpreted();  // Single-frame deoptimization triggered.
            caught = true;
        }
        if (!caught) {
            System.out.println("Expected exception");
        }
        assertIsInterpreted();
        return 79.3d;
    }

    public static float $noinline$run3A() {
        assertIsManaged();
        // Prevent inlining.
        if (sFlag) {
            throw new Error();
        }
        // Deoptimize callers.
        deoptimizeAll();
        assertIsInterpreted();
        assertCallerIsInterpreted();  // $noinline$run3B is deoptimizeable.
        return 0.034f;
    }

    public static float $noinline$run3B() {
        assertIsManaged();
        // Prevent inlining.
        if (sFlag) {
            throw new Error();
        }
        float res = $noinline$run3A();
        assertIsInterpreted();
        return res;
    }
}
