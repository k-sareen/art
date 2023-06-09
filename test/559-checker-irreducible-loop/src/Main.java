/*
 * Copyright (C) 2015 The Android Open Source Project
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

public class Main {
  public static void main(String[] args) throws Exception {
    Class<?> c = Class.forName("IrreducibleLoop");
    {
      Method m = c.getMethod("simpleLoop", int.class);
      Object[] arguments = { 42 };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("lse", int.class, Main.class);
      Object[] arguments = { 42, new Main() };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("dce", int.class);
      Object[] arguments = { 42 };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("liveness", int.class, int.class);
      Object[] arguments = { 42, 42 };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("gvn");
      Object[] arguments = { };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("licm1", int.class);
      Object[] arguments = { 42 };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("licm2", int.class);
      Object[] arguments = { 42 };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("testDoNotInlineIrreducible", int.class);
      Object[] arguments = { 42 };
      System.out.println(m.invoke(null, arguments));
    }

    {
      Method m = c.getMethod("testDoNotInlineIrreducible", int.class);
      Object[] arguments = { 0 };
      System.out.println(m.invoke(null, arguments));
    }
  }

  int myField;
}

class Other {
}
