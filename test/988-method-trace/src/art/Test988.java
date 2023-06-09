/*
 * Copyright (C) 2017 The Android Open Source Project
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

import java.io.PrintWriter;
import java.io.StringWriter;
import java.lang.reflect.Executable;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Proxy;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;
import java.util.function.IntUnaryOperator;
import java.util.function.Predicate;

public class Test988 {

    // Methods with non-deterministic output that should not be printed.
    static List<Predicate<Executable>> NON_DETERMINISTIC_OUTPUT_METHODS = new ArrayList<>();
    static List<Predicate<Executable>> NON_DETERMINISTIC_OUTPUT_TYPE_METHODS = new ArrayList<>();
    static List<Class<?>> NON_DETERMINISTIC_TYPE_NAMES = new ArrayList<>();

    static Predicate<Executable> IS_NON_DETERMINISTIC_OUTPUT =
        (x) -> NON_DETERMINISTIC_OUTPUT_METHODS.stream().anyMatch((pred) -> pred.test(x));
    static Predicate<Executable> IS_NON_DETERMINISTIC_OUTPUT_TYPE =
        (x) -> NON_DETERMINISTIC_OUTPUT_TYPE_METHODS.stream().anyMatch((pred) -> pred.test(x));

    public static final Predicate<Executable> EqPred(Executable m) {
      return (Executable n) -> n.equals(m);
    }

    static {
      // Throwable.nativeFillInStackTrace is only on android and hiddenapi so we
      // should avoid trying to find it at all.
      NON_DETERMINISTIC_OUTPUT_METHODS.add(
        (Executable ex) -> {
          return ex.getDeclaringClass().equals(Throwable.class)
              && ex.getName().equals("nativeFillInStackTrace");
        });
      try {
        NON_DETERMINISTIC_OUTPUT_METHODS.add(
            EqPred(Thread.class.getDeclaredMethod("currentThread")));
        NON_DETERMINISTIC_OUTPUT_TYPE_METHODS.add(
            EqPred(Thread.class.getDeclaredMethod("currentThread")));
      } catch (Exception e) {}
      try {
        NON_DETERMINISTIC_TYPE_NAMES.add(
            Proxy.getProxyClass(Test988.class.getClassLoader(), new Class[] { Runnable.class }));
      } catch (Exception e) {}
    }

    static interface Printable {
        public void Print();
    }

    static final class MethodEntry implements Printable {
        private Executable m;
        private int cnt;
        public MethodEntry(Executable m, int cnt) {
            this.m = m;
            this.cnt = cnt;
        }
        @Override
        public void Print() {
            System.out.println(whitespace(cnt) + "=> " + methodToString(m));
        }
    }

    private static String genericToString(Object val) {
      if (val == null) {
        return "null";
      } else if (val.getClass().isArray()) {
        return arrayToString(val);
      } else if (val instanceof Throwable) {
        StringWriter w = new StringWriter();
        Throwable thr = ((Throwable) val);
        w.write(thr.getClass().getName() + ": " + thr.getMessage() + "\n");
        for (StackTraceElement e : thr.getStackTrace()) {
          if (e.getClassName().startsWith("art.")) {
            w.write("\t" + e + "\n");
          } else {
            w.write("\t<additional hidden frames>\n");
            break;
          }
        }
        return w.toString();
      } else {
        return val.toString();
      }
    }

    private static String charArrayToString(char[] src) {
      String[] res = new String[src.length];
      for (int i = 0; i < src.length; i++) {
        if (Character.isISOControl(src[i])) {
          res[i] = Character.getName(src[i]);
        } else {
          res[i] = Character.toString(src[i]);
        }
      }
      return Arrays.toString(res);
    }

    private static String arrayToString(Object val) {
      Class<?> klass = val.getClass();
      if ((new Object[0]).getClass().isAssignableFrom(klass)) {
        return Arrays.toString(
            Arrays.stream((Object[])val).map(new Function<Object, String>() {
              public String apply(Object o) {
                return Test988.genericToString(o);
              }
            }).toArray());
      } else if ((new byte[0]).getClass().isAssignableFrom(klass)) {
        return Arrays.toString((byte[])val);
      } else if ((new char[0]).getClass().isAssignableFrom(klass)) {
        return charArrayToString((char[])val);
      } else if ((new short[0]).getClass().isAssignableFrom(klass)) {
        return Arrays.toString((short[])val);
      } else if ((new int[0]).getClass().isAssignableFrom(klass)) {
        return Arrays.toString((int[])val);
      } else if ((new long[0]).getClass().isAssignableFrom(klass)) {
        return Arrays.toString((long[])val);
      } else if ((new float[0]).getClass().isAssignableFrom(klass)) {
        return Arrays.toString((float[])val);
      } else if ((new double[0]).getClass().isAssignableFrom(klass)) {
        return Arrays.toString((double[])val);
      } else {
        throw new Error("Unknown type " + klass);
      }
    }

    static String methodToString(Executable m) {
      // Make the output more similar between ART and RI,
      // by removing the 'native' specifier from methods.
      String methodStr;
      if (NON_DETERMINISTIC_TYPE_NAMES.contains(m.getDeclaringClass())) {
        methodStr = m.toString().replace(m.getDeclaringClass().getName(),
            "<non-deterministic-type " +
            NON_DETERMINISTIC_TYPE_NAMES.indexOf(m.getDeclaringClass()) +
            ">");
      } else {
        methodStr = m.toString();
      }
      return methodStr.replaceFirst(" native", "");
    }

    static final class MethodReturn implements Printable {
        private Executable m;
        private Object val;
        private int cnt;
        public MethodReturn(Executable m, Object val, int cnt) {
            this.m = m;
            this.val = val;
            this.cnt = cnt;
        }
        @Override
        public void Print() {
            String print;
            if (IS_NON_DETERMINISTIC_OUTPUT.test(m)) {
                print = "<non-deterministic>";
            } else {
                print = genericToString(val);
            }
            Class<?> klass = null;
            if (val != null) {
              klass = val.getClass();
            }
            String klass_print;
            if (klass == null) {
              klass_print =  "null";
            } else if (NON_DETERMINISTIC_TYPE_NAMES.contains(klass)) {
              klass_print = "<non-deterministic-class " +
                  NON_DETERMINISTIC_TYPE_NAMES.indexOf(klass) + ">";
            } else if (IS_NON_DETERMINISTIC_OUTPUT_TYPE.test(m)) {
              klass_print = "<non-deterministic>";
            } else {
              klass_print = klass.toString();
            }
            System.out.println(
                whitespace(cnt) + "<= " + methodToString(m) + " -> <" + klass_print + ": " + print + ">");
        }
    }

    static final class MethodThrownThrough implements Printable {
        private Executable m;
        private int cnt;
        public MethodThrownThrough(Executable m, int cnt) {
            this.m = m;
            this.cnt = cnt;
        }
        @Override
        public void Print() {
            System.out.println(whitespace(cnt) + "<= " + methodToString(m) + " EXCEPTION");
        }
    }

    private static String whitespace(int n) {
      String out = "";
      while (n > 0) {
        n--;
        out += ".";
      }
      return out;
    }

    static final class FibThrow implements Printable {
        private String format;
        private int arg;
        private Throwable res;
        public FibThrow(String format, int arg, Throwable res) {
            this.format = format;
            this.arg = arg;
            this.res = res;
        }

        @Override
        public void Print() {
            System.out.printf(format, arg, genericToString(res));
        }
    }

    static final class FibResult implements Printable {
        private String format;
        private int arg;
        private int res;
        public FibResult(String format, int arg, int res) {
            this.format = format;
            this.arg = arg;
            this.res = res;
        }

        @Override
        public void Print() {
            System.out.printf(format, arg, res);
        }
    }

    private static ArrayList<Printable> results = new ArrayList<>();
    private static int results_index = 0;
    // Starts with => enableMethodTracing
    //             .=> enableTracing
    private static int cnt = 2;

    static void addToResults(Printable obj) {
      // Reserve space for the current object. If any other method entry callbacks are called they
      // will reserve more space. Without this we may get into strange problems where ArrayList::add
      // cecks there is enough space (which involves a couple of method calls) which then use up the
      // space and by the time we actually add this record there is no capacity left.
      results_index++;
      results.ensureCapacity(results_index + 1);
      results.add(obj);
    }

    // Iterative version
    static final class IterOp implements IntUnaryOperator {
      public int applyAsInt(int x) {
        return iter_fibonacci(x);
      }
    }
    static int iter_fibonacci(int n) {
        if (n < 0) {
            throw new Error("Bad argument: " + n + " < 0");
        } else if (n == 0) {
            return 0;
        }
        int x = 1;
        int y = 1;
        for (int i = 3; i <= n; i++) {
            int z = x + y;
            x = y;
            y = z;
        }
        return y;
    }

    // Recursive version
    static final class RecurOp implements IntUnaryOperator {
      public int applyAsInt(int x) {
        return fibonacci(x);
      }
    }
    static int fibonacci(int n) {
        if (n < 0) {
            throw new Error("Bad argument: " + n + " < 0");
        } else if ((n == 0) || (n == 1)) {
            return n;
        } else {
            return fibonacci(n - 1) + (fibonacci(n - 2));
        }
    }

    static final class NativeOp implements IntUnaryOperator {
      public int applyAsInt(int x) {
        return nativeFibonacci(x);
      }
    }
    static native int nativeFibonacci(int n);

    static final class TestRunnableInvokeHandler implements InvocationHandler {
      public Object invoke(Object proxy, Method m, Object[] args) throws Throwable {
        return null;
      }
    }

    static final int METHOD_TRACING_IGNORE_DEPTH = 2;
    static boolean sMethodTracingIgnore = false;

    public static void notifyMethodEntry(Executable m) {
        // Called by native code when a method is entered. This method is ignored by the native
        // entry and exit hooks.
        cnt++;
        if ((cnt - 1) > METHOD_TRACING_IGNORE_DEPTH && sMethodTracingIgnore) {
          return;
        }
        addToResults(new MethodEntry(m, cnt - 1));
    }

    public static void notifyMethodExit(Executable m, boolean exception, Object result) {
        cnt--;

        if (cnt > METHOD_TRACING_IGNORE_DEPTH && sMethodTracingIgnore) {
          return;
        }

        if (exception) {
            addToResults(new MethodThrownThrough(m, cnt));
        } else {
            addToResults(new MethodReturn(m, result, cnt));
        }
    }

    public static void run() throws Exception {
        // call this here so it is linked. It doesn't actually do anything here.
        loadAllClasses();
        Trace.disableTracing(Thread.currentThread());
        // Call this prior to starting tracing since its implementation is so deep into reflection
        // that it will be changing all the time and difficult to keep up with.
        Runnable runnable = (Runnable)Proxy.newProxyInstance(
                    Test988.class.getClassLoader(),
                    new Class[]{ Runnable.class },
                    new TestRunnableInvokeHandler());
        Trace.enableMethodTracing(
            Test988.class,
            Test988.class.getDeclaredMethod("notifyMethodEntry", Executable.class),
            Test988.class.getDeclaredMethod(
                "notifyMethodExit", Executable.class, Boolean.TYPE, Object.class),
            Thread.currentThread());
        doFibTest(30, new IterOp());
        doFibTest(5, new RecurOp());
        doFibTest(5, new NativeOp());
        doFibTest(-19, new IterOp());
        doFibTest(-19, new RecurOp());
        doFibTest(-19, new NativeOp());

        runnable.run();

        sMethodTracingIgnore = true;
        IntrinsicsTest.doTest();
        sMethodTracingIgnore = false;
        // Turn off method tracing so we don't have to deal with print internals.
        Trace.disableTracing(Thread.currentThread());
        printResults();
    }

    // This ensures that all classes we touch are loaded before we start recording traces. This
    // eliminates a major source of divergence between the RI and ART.
    public static void loadAllClasses() {
      MethodThrownThrough.class.toString();
      MethodEntry.class.toString();
      MethodReturn.class.toString();
      FibResult.class.toString();
      FibThrow.class.toString();
      Printable.class.toString();
      ArrayList.class.toString();
      RecurOp.class.toString();
      IterOp.class.toString();
      NativeOp.class.toString();
      StringBuilder.class.toString();
      Runnable.class.toString();
      TestRunnableInvokeHandler.class.toString();
      Proxy.class.toString();
      Proxy.getProxyClass(
          Test988.class.getClassLoader(), new Class[] { Runnable.class }).toString();
      IntrinsicsTest.initialize();  // ensure <clinit> is executed prior to tracing.
    }

    public static void printResults() {
        for (Printable p : results) {
            p.Print();
        }
    }

    public static void doFibTest(int x, IntUnaryOperator op) {
      try {
        int y = op.applyAsInt(x);
        addToResults(new FibResult("fibonacci(%d)=%d\n", x, y));
      } catch (Throwable t) {
        addToResults(new FibThrow("fibonacci(%d) -> %s\n", x, t));
      }
    }

    static class IntrinsicsTest {
      static int[] sSourceArray = { 0, 1, 2, 3, 4, 5 };
      static int[] sDestArray =   { 5, 6, 7, 8, 9, 10 };

      static char[] sSourceArrayChar = { '0', '1', '2', '3', '4', '5' };
      static char[] sDestArrayChar =   { '5', '6', '7', '8', '9', 'a' };

      static void initialize() {
        Test988Intrinsics.initialize();

        // Pre-load all classes used in #doTest manual intrinsics.
        java.lang.System.class.toString();
      }
      static void doTest() {
        // Ensure that the ART intrinsics in intrinsics_list.h are also being traced,
        // since in non-tracing operation they are effectively inlined by the optimizing compiler.

        // Auto-generated test file that uses null/0s as default parameters.
        Test988Intrinsics.test();

        // Manual list here for functions that require special non-null/non-zero parameters:
        System.arraycopy(sSourceArray, 0, sDestArray, 0, 1);
        System.arraycopy(sSourceArrayChar, 0, sDestArrayChar, 0, 1);
      }
    }
}
