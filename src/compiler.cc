// Copyright 2011 Google Inc. All Rights Reserved.

#include "compiler.h"

#include "assembler.h"
#include "class_linker.h"
#include "dex_cache.h"
#include "jni_compiler.h"

extern bool oatCompileMethod(art::Method*, art::InstructionSet);

namespace art {

// TODO need to specify target
const ClassLoader* Compiler::Compile(std::vector<const DexFile*> class_path) {
  const ClassLoader* class_loader = PathClassLoader::Alloc(class_path);
  Resolve(class_loader);
  for (size_t i = 0; i != class_path.size(); ++i) {
    const DexFile* dex_file = class_path[i];
    CHECK(dex_file != NULL);
    CompileDexFile(class_loader, *dex_file);
  }
  return class_loader;
}

void Compiler::Resolve(const ClassLoader* class_loader) {
  const std::vector<const DexFile*>& class_path = class_loader->GetClassPath();
  for (size_t i = 0; i != class_path.size(); ++i) {
    const DexFile* dex_file = class_path[i];
    CHECK(dex_file != NULL);
    ResolveDexFile(class_loader, *dex_file);
  }
}

void Compiler::ResolveDexFile(const ClassLoader* class_loader, const DexFile& dex_file) {
  ClassLinker* class_linker = Runtime::Current()->GetClassLinker();

  // Strings are easy, they always are simply resolved to literals in the same file
  DexCache* dex_cache = class_linker->FindDexCache(dex_file);
  for (size_t i = 0; i < dex_cache->NumStrings(); i++) {
    class_linker->ResolveString(dex_file, i, dex_cache);
  }

  // Class derived values are more complicated, they require the linker and loader
  for (size_t i = 0; i < dex_cache->NumTypes(); i++) {
    class_linker->ResolveType(dex_file, i, dex_cache, class_loader);
  }
  for (size_t i = 0; i < dex_cache->NumMethods(); i++) {
    // TODO: move resolution into compiler proper where we will know method_type
    int method_type = 0;
    class_linker->ResolveMethod(dex_file, i, dex_cache, class_loader, method_type);
  }
  for (size_t i = 0; i < dex_cache->NumFields(); i++) {
    // TODO: move resolution into compiler proper where we will know is_static
    bool is_static = false;
    class_linker->ResolveField(dex_file, i, dex_cache, class_loader, is_static);
  }
}

void Compiler::CompileDexFile(const ClassLoader* class_loader, const DexFile& dex_file) {
  ClassLinker* class_linker = Runtime::Current()->GetClassLinker();
  for (size_t i = 0; i < dex_file.NumClassDefs(); i++) {
    const DexFile::ClassDef& class_def = dex_file.GetClassDef(i);
    const char* descriptor = dex_file.GetClassDescriptor(class_def);
    Class* klass = class_linker->FindClass(descriptor, class_loader);
    CHECK(klass != NULL);
    CompileClass(klass);
  }
}

void Compiler::CompileClass(Class* klass) {
  for (size_t i = 0; i < klass->NumDirectMethods(); i++) {
    CompileMethod(klass->GetDirectMethod(i));
  }
  for (size_t i = 0; i < klass->NumVirtualMethods(); i++) {
    CompileMethod(klass->GetVirtualMethod(i));
  }
}

void Compiler::CompileMethod(Method* method) {
  if (method->IsNative()) {
    // TODO note code will be unmapped when JniCompiler goes out of scope
    Assembler jni_asm;
    JniCompiler jni_compiler;
    jni_compiler.Compile(&jni_asm, method);
  } else if (method->IsAbstract()) {
    // TODO: This might be also noted in the ClassLinker.
    // Probably makes more sense to do here?
    UNIMPLEMENTED(FATAL) << "compile stub to throw AbstractMethodError";
  } else {
    oatCompileMethod(method, kThumb2);
  }
  // CHECK(method->HasCode());  // TODO: enable this check ASAP
}

}  // namespace art
