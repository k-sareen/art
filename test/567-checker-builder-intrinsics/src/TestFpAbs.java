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

/**
 * A few tests of Math.abs for floating-point data.
 *
 * Note, as a "quality of implementation", rather than pure "spec compliance",
 * we require that Math.abs() clears the sign bit (but changes nothing else)
 * for all numbers, including NaN (signaling NaN may become quiet though).
 */
public class TestFpAbs {

  private final static boolean isDalvik =
      System.getProperty("java.vm.name").equals("Dalvik");

  private static final int SPQUIET = 1 << 22;
  private static final long DPQUIET = 1L << 51;

  /// CHECK-START: float TestFpAbs.$opt$noinline$absSP(float) builder (after)
  /// CHECK-DAG: <<Result:f\d+>> Abs
  /// CHECK-DAG:                 Return [<<Result>>]
  private static float $opt$noinline$absSP(float f) {
    return Math.abs(f);
  }

  /// CHECK-START: double TestFpAbs.$opt$noinline$absDP(double) builder (after)
  /// CHECK-DAG: <<Result:d\d+>> Abs
  /// CHECK-DAG:                 Return [<<Result>>]
  private static double $opt$noinline$absDP(double d) {
    return Math.abs(d);
  }

  public static void main() {
    // A few obvious numbers.
    for (float f = -100.0f; f < 0.0f; f += 0.5f) {
      expectEqualsSP(-f, $opt$noinline$absSP(f));
    }
    for (float f = 0.0f; f <= 100.0f; f += 0.5f) {
      expectEqualsSP(f, $opt$noinline$absSP(f));
    }
    for (float f = -1.5f; f <= -1.499f; f = Math.nextAfter(f, Float.POSITIVE_INFINITY)) {
      expectEqualsSP(-f, $opt$noinline$absSP(f));
    }
    for (float f = 1.499f; f <= 1.5f; f = Math.nextAfter(f, Float.POSITIVE_INFINITY)) {
      expectEqualsSP(f, $opt$noinline$absSP(f));
    }

    // Zero
    expectEquals32(0, Float.floatToRawIntBits($opt$noinline$absSP(+0.0f)));
    expectEquals32(0, Float.floatToRawIntBits($opt$noinline$absSP(-0.0f)));

    // Inf.
    expectEqualsSP(Float.POSITIVE_INFINITY, $opt$noinline$absSP(Float.NEGATIVE_INFINITY));
    expectEqualsSP(Float.POSITIVE_INFINITY, $opt$noinline$absSP(Float.POSITIVE_INFINITY));

    // A few NaN numbers.
    int[] spnans = {
      0x7f800001,  // signaling
      0x7fa00000,
      0x7fbfffff,
      0x7fc00000,  // quiet
      0x7fc00001,
      0x7fffffff,
      0xff800001,  // signaling
      0xffa00000,
      0xffbfffff,
      0xffc00000,  // quiet
      0xffffffff
    };
    for (int i = 0; i < spnans.length; i++) {
      float f = Float.intBitsToFloat(spnans[i]);
      expectEqualsNaN32(
          spnans[i] & Integer.MAX_VALUE,
          Float.floatToRawIntBits($opt$noinline$absSP(f)));
    }

    // A few obvious numbers.
    for (double d = -100.0; d < 0.0; d += 0.5) {
      expectEqualsDP(-d, $opt$noinline$absDP(d));
    }
    for (double d = 0.0; d <= 100.0; d += 0.5) {
      expectEqualsDP(d, $opt$noinline$absDP(d));
    }
    for (double d = -1.5d; d <= -1.49999999999d; d = Math.nextAfter(d, Double.POSITIVE_INFINITY)) {
      expectEqualsDP(-d, $opt$noinline$absDP(d));
    }
    for (double d = 1.49999999999d; d <= 1.5; d = Math.nextAfter(d, Double.POSITIVE_INFINITY)) {
      expectEqualsDP(d, $opt$noinline$absDP(d));
    }

    // Zero
    expectEquals64(0L, Double.doubleToRawLongBits($opt$noinline$absDP(+0.0f)));
    expectEquals64(0L, Double.doubleToRawLongBits($opt$noinline$absDP(-0.0f)));

    // Inf.
    expectEqualsDP(Double.POSITIVE_INFINITY, $opt$noinline$absDP(Double.NEGATIVE_INFINITY));
    expectEqualsDP(Double.POSITIVE_INFINITY, $opt$noinline$absDP(Double.POSITIVE_INFINITY));

    // A few NaN numbers.
    long[] dpnans = {
      0x7ff0000000000001L,
      0x7ff4000000000000L,
      0x7ff8000000000000L,
      0x7fffffffffffffffL,
      0xfff0000000000001L,
      0xfff4000000000000L,
      0xfff8000000000000L,
      0xffffffffffffffffL
    };
    for (int i = 0; i < dpnans.length; i++) {
      double d = Double.longBitsToDouble(dpnans[i]);
      expectEqualsNaN64(
          dpnans[i] & Long.MAX_VALUE,
          Double.doubleToRawLongBits($opt$noinline$absDP(d)));
    }

    System.out.println("TestFpAbs passed");
  }

  private static void expectEquals32(int expected, int result) {
    if (expected != result) {
      throw new Error("Expected: 0x" + Integer.toHexString(expected)
          + ", found: 0x" + Integer.toHexString(result));
    }
  }

  // We allow that an expected NaN result has become quiet.
  private static void expectEqualsNaN32(int expected, int result) {
    if (expected != result && (expected | SPQUIET) != result) {
      if (!isDalvik) {
        // If not on ART, relax the expected value more towards just
        // "spec compliance" and allow sign bit to remain set for NaN.
        if (expected == (result & Integer.MAX_VALUE)) {
          return;
        }
      }
      throw new Error("Expected: 0x" + Integer.toHexString(expected)
          + ", found: 0x" + Integer.toHexString(result));
    }
  }

  private static void expectEquals64(long expected, long result) {
    if (expected != result) {
      throw new Error("Expected: 0x" + Long.toHexString(expected)
          + ", found: 0x" + Long.toHexString(result));
    }
  }

  // We allow that an expected NaN result has become quiet.
  private static void expectEqualsNaN64(long expected, long result) {
    if (expected != result && (expected | DPQUIET) != result) {
      if (!isDalvik) {
        // If not on ART, relax the expected value more towards just
        // "spec compliance" and allow sign bit to remain set for NaN.
        if (expected == (result & Long.MAX_VALUE)) {
          return;
        }
      }
      throw new Error("Expected: 0x" + Long.toHexString(expected)
          + ", found: 0x" + Long.toHexString(result));
    }
  }

  private static void expectEqualsSP(float expected, float result) {
    if (expected != result) {
      throw new Error("Expected: " + expected + ", found: " + result);
    }
  }

  private static void expectEqualsDP(double expected, double result) {
    if (expected != result) {
      throw new Error("Expected: " + expected + ", found: " + result);
    }
  }
}
