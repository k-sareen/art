%def op_check_cast():
    unimp

%def op_check_cast_slow_path():
    unimp

%def op_instance_of():
    unimp

%def op_instance_of_slow_path():
    unimp

%def op_new_instance():
    unimp

// *** iget ***

%def op_iget(load="", wide="", is_object=""):
    unimp

%def op_iget_slow_path(load, wide, is_object):
    unimp

%def op_iget_boolean():
    unimp

%def op_iget_byte():
    unimp

%def op_iget_char():
    unimp

%def op_iget_short():
    unimp

%def op_iget_wide():
    unimp

%def op_iget_object():
    unimp

// *** iput ***

%def op_iput(wide="", is_object=""):
    unimp

%def op_iput_slow_path(wide, is_object):
    unimp

%def op_iput_boolean():
    unimp

%def op_iput_byte():
    unimp

%def op_iput_char():
    unimp

%def op_iput_short():
    unimp

%def op_iput_wide():
    unimp

%def op_iput_object():
    unimp

// *** sget ***

// sget vAA, field@BBBB
// Format 21c: AA|60 BBBB
%def op_sget(width="32", zext=False):
   // Fast path: NterpGetStaticField's resolved_field from thread-local cache.
   // Stores cache value in a0 to match slow path's return from NterpGetStaticField.
   FETCH_FROM_THREAD_CACHE /*resolved_field*/a0, .L${opcode}_slow, t0, t1
.L${opcode}_slow_resume:

   lwu t0, ART_FIELD_OFFSET_OFFSET(a0)           // t0 := field offset
   lwu a0, ART_FIELD_DECLARING_CLASS_OFFSET(a0)  // a0 := holder
   srliw t1, xINST, 8                            // t1 := AA
   TEST_IF_MARKING t2, .L${opcode}_mark
.L${opcode}_mark_resume:

   add t0, t0, a0      // t0 := field addr, after possible a0 update
%  if width == "8" and zext:
     lbu t2, (t0)
%  elif width == "8":
     lb t2, (t0)
%  elif width == "16" and zext:
     lhu t2, (t0)
%  elif width == "16":
     lh t2, (t0)
%  elif width == "32":
     lw t2, (t0)
%  else:  # width = 64:
     ld t2, (t0)
%#:
   FETCH_ADVANCE_INST 2
%  is_wide = "1" if width == "64" else "0"
   SET_VREG t2, t1, z0=t0, is_wide=$is_wide
   GET_INST_OPCODE t0
   GOTO_OPCODE t0

.L${opcode}_mark:
   call art_quick_read_barrier_mark_reg10  // a0, holder
   j .L${opcode}_mark_resume

.L${opcode}_slow:
   mv a0, xSELF
   ld a1, (sp)  // a1 := caller ArtMethod*
   mv a2, xPC
   mv a3, zero
   EXPORT_PC
   call nterp_get_static_field  // result a0 := resolved_field

   // Test for volatile bit
   slli t0, a0, 63
   bgez t0, .L${opcode}_slow_resume
%  volatile_path = add_slow_path(op_sget_volatile, width, zext, "t0", "t1", "t2")
   tail $volatile_path


// Volatile static store.
// Temporaries: z0, z1, z2
%def op_sget_volatile(width, zext, z0, z1, z2):
   CLEAR_STATIC_VOLATILE_MARKER a0
   lwu $z0, ART_FIELD_OFFSET_OFFSET(a0)          // z0 := field offset
   lwu a0, ART_FIELD_DECLARING_CLASS_OFFSET(a0)  // a0 := holder
   srliw $z1, xINST, 8                           // z1 := AA
   TEST_IF_MARKING $z2, .L${opcode}_volatile_mark
.L${opcode}_volatile_mark_resume:

   add $z0, $z0, a0     // z0 := field addr, after possible a0 update
   // Atomic load: "fence rw,rw ; LOAD ; fence r,rw"
   fence rw, rw
%  if width == "8" and zext:
     lbu $z2, ($z0)
%  elif width == "8":
     lb $z2, ($z0)
%  elif width == "16" and zext:
     lhu $z2, ($z0)
%  elif width == "16":
     lh $z2, ($z0)
%  elif width == "32":
     lw $z2, ($z0)
%  else:  # width = 64:
     ld $z2, ($z0)
%#:
   fence r, rw

   FETCH_ADVANCE_INST 2
%  is_wide = "1" if width == "64" else "0"
   SET_VREG $z2, $z1, z0=$z0, is_wide=$is_wide
   GET_INST_OPCODE $z0
   GOTO_OPCODE $z0

.L${opcode}_volatile_mark:
   call art_quick_read_barrier_mark_reg10  // a0, holder
   j .L${opcode}_volatile_mark_resume


// sget-wide vAA, field@BBBB
// Format 21c: AA|61 BBBB
%def op_sget_wide():
%  op_sget(width="64")


// sget-object vAA, field@BBBB
// Format 21c: AA|62 BBBB
// Variant for object load contains extra logic for GC mark.
%def op_sget_object():
   // Fast path: NterpGetStaticField's resolved_field from thread-local cache.
   // Stores cache value in a0 to match slow path's return from NterpGetStaticField.
   FETCH_FROM_THREAD_CACHE /*resolved_field*/a0, .L${opcode}_slow, t0, t1
.L${opcode}_slow_resume:

   lwu t0, ART_FIELD_OFFSET_OFFSET(a0)           // t0 := field offset
   lwu a0, ART_FIELD_DECLARING_CLASS_OFFSET(a0)  // a0 := holder
   srliw t1, xINST, 8                            // t1 := AA
   TEST_IF_MARKING t2, .L${opcode}_mark

   add t0, t0, a0      // t0 := field addr
   lwu a0, (t0)        // a0 := value (ref)

.L${opcode}_mark_resume:
   FETCH_ADVANCE_INST 2
   SET_VREG_OBJECT a0, t1, z0=t0
   GET_INST_OPCODE t0
   GOTO_OPCODE t0

.L${opcode}_mark:
   call art_quick_read_barrier_mark_reg10  // a0, holder
   add t0, t0, a0      // t0 := field addr, after a0 update
   lwu a0, (t0)        // a0 := value (ref)
   call art_quick_read_barrier_mark_reg10  // a0, object
   j .L${opcode}_mark_resume

.L${opcode}_slow:
%  slow_path = add_slow_path(op_sget_object_slow_path, "t0", "t1", "t2")
   tail $slow_path


// Static store, object variant. Contains both slow path and volatile path
// due to handler size limitation in op_sget_object.
// Hardcoded: a0, a1, a2, a3, xSELF, xPC, xINST, xFP, xREFS
// Temporaries: z0, z1, z2
%def op_sget_object_slow_path(z0, z1, z2):
   mv a0, xSELF
   ld a1, (sp)  // a1 := caller ArtMethod*
   mv a2, xPC
   mv a3, zero
   EXPORT_PC
   call nterp_get_static_field  // result a0 := resolved_field

   // Test for volatile bit
   slli $z0, a0, 63
   bltz $z0, .L${opcode}_volatile
   tail .L${opcode}_slow_resume  // resume offset exceeds branch imm

.L${opcode}_volatile:
   CLEAR_STATIC_VOLATILE_MARKER a0
   lwu $z0, ART_FIELD_OFFSET_OFFSET(a0)          // z0 := field offset
   lwu a0, ART_FIELD_DECLARING_CLASS_OFFSET(a0)  // a0 := holder
   srliw $z1, xINST, 8                           // z1 := AA
   TEST_IF_MARKING $z2, .L${opcode}_volatile_mark

   add $z0, $z0, a0  // z0 := field addr
   fence rw, rw
   lwu a0, ($z0)  // Atomic ref load: "fence rw,rw, ; LOAD ; fence r,rw"
   fence r, rw

.L${opcode}_volatile_mark_resume:
   FETCH_ADVANCE_INST 2
   SET_VREG_OBJECT a0, $z1, z0=$z2
   GET_INST_OPCODE $z0
   GOTO_OPCODE $z0

.L${opcode}_volatile_mark:
   call art_quick_read_barrier_mark_reg10  // a0, holder
   add $z0, $z0, a0  // z0 := field addr, after a0 update
   fence rw, rw
   lwu a0, ($z0)  // Atomic ref load: "fence rw,rw, ; LOAD ; fence r,rw"
   fence r, rw
   call art_quick_read_barrier_mark_reg10  // a0, object
   j .L${opcode}_volatile_mark_resume


// sget-boolean vAA, field@BBBB
// Format 21c: AA|63 BBBB
%def op_sget_boolean():
%  op_sget(width="8", zext=True)


// sget-byte vAA, field@BBBB
// Format 21c: AA|64 BBBB
%def op_sget_byte():
%  op_sget(width="8")


// sget-char vAA, field@BBBB
// Format 21c: AA|65 BBBB
%def op_sget_char():
%  op_sget(width="16", zext=True)


// sget-short vAA, field@BBBB
// Format 21c: AA|66 BBBB
%def op_sget_short():
%  op_sget(width="16")


// *** sput ***

.macro CLEAR_STATIC_VOLATILE_MARKER reg
    andi \reg, \reg, ~0x1
.endm

// sput vAA, field@BBBB
// Format 21c: AA|67 BBBB
// Clobbers: t0, t1, t2, a0
%def op_sput(width="32", is_object=False):
   srliw t2, xINST, 8  // t2 := AA
%  if width == "64":
     GET_VREG_WIDE s7, t2
%  elif is_object:
     GET_VREG_OBJECT s7, t2
%  else:
     GET_VREG s7, t2
%#:
   // Fast path: NterpGetStaticField's resolved_field from thread-local cache.
   // Stores cache value in a0 to match slow path's return from NterpGetStaticField.
   // Slow path: updates s7 if is_object, for possible GC movement.
   FETCH_FROM_THREAD_CACHE /*resolved_field*/a0, .L${opcode}_slow, t0, t1

.L${opcode}_slow_resume:
   lwu t0, ART_FIELD_OFFSET_OFFSET(a0)
   lwu a0, ART_FIELD_DECLARING_CLASS_OFFSET(a0)  // a0 := holder
   TEST_IF_MARKING t1, .L${opcode}_mark

.L${opcode}_mark_resume:
   add t0, t0, a0
%  if width == "8":
     sb s7, (t0)
%  elif width == "16":
     sh s7, (t0)
%  elif width == "32":
     sw s7, (t0)
%  else:  # width 64:
     sd s7, (t0)
%#:
%  write_barrier_if_object(is_object=is_object, value="s7", holder="a0", z0="t0", z1="t1", uniq=f"{opcode}")
   FETCH_ADVANCE_INST 2
   GET_INST_OPCODE t0
   GOTO_OPCODE t0

.L${opcode}_slow:
%  slow_path = add_slow_path(op_sput_slow_path, width, is_object, "s7", "t0", "t1")
   tail $slow_path  // slow path offset exceeds regular branch imm in FETCH_FROM_THREAD_CACHE
                    // return a0 := resolved_field
.L${opcode}_mark:
   call art_quick_read_barrier_mark_reg10  // a0
   j .L${opcode}_mark_resume


// Input
//   - width: bit width of value. One of 8, 16, 32, 64.
//   - is_object: True if reference type
//   - value: register holding value to store. Avoid clobber set.
// Output
//   - a0: holds resolved_field.
// Hardcoded: a0, a1, a2, xREFS, xINST, xPC
// Temporaries z0, z1
%def op_sput_slow_path(width, is_object, value, z0, z1):
   // Args for nterp_get_static_field
   mv a0, xSELF
   ld a1, (sp)
   mv a2, xPC
%  if is_object:
     mv a3, $value
%  else:
     mv a3, zero
%#:
   EXPORT_PC
   call nterp_get_static_field  // result a0 := resolved_field
%  if is_object:
     // Reload value, it may have moved.
     srliw $value, xINST, 8  // value:= AA
     GET_VREG_OBJECT $value, $value // value:= v[AA]
%#:
   // Test for volatile bit
   slli $z0, a0, 63
   bltz $z0, 1f
   tail .L${opcode}_slow_resume  // resume offset exceeds branch imm
1:
   // Volatile static store.
   CLEAR_STATIC_VOLATILE_MARKER a0
   lwu $z0, ART_FIELD_OFFSET_OFFSET(a0)
   lwu a0, ART_FIELD_DECLARING_CLASS_OFFSET(a0)  // a0 := holder
   TEST_IF_MARKING $z1, 3f
2:
   add $z0, a0, $z0
   // Ensure the volatile store is released.
%  if width == "8":
     fence rw, w
     sb $value, ($z0)
     fence rw, rw
%  elif width == "16":
     fence rw, w
     sh $value, ($z0)
     fence rw, rw
%  elif width == "32":
     // \value must NOT be the destination register, the destination gets clobbered!
     // For refs, \value's original value is used in the write barrier below.
     amoswap.w.rl zero, $value, ($z0)
%  else:  # width == 64:
     amoswap.d.rl zero, $value, ($z0)
%#:
%  write_barrier_if_object(is_object=is_object, value=value, holder="a0", z0=z0, z1=z1, uniq=f"slow_{opcode}")
   FETCH_ADVANCE_INST 2
   GET_INST_OPCODE $z0
   GOTO_OPCODE $z0
3:
   call art_quick_read_barrier_mark_reg10  // a0
   j 2b


%def write_barrier_if_object(is_object, value, holder, z0, z1, uniq):
%  if is_object:
     beqz $value, .L${uniq}_skip_write_barrier  // No object, skip out.
     ld $z0, THREAD_CARD_TABLE_OFFSET(xSELF)
     srli $z1, $holder, CARD_TABLE_CARD_SHIFT
     add $z1, $z0, $z1
     sb $z0, ($z1)
.L${uniq}_skip_write_barrier:


// sput-wide vAA, field@BBBB
// Format 21c: AA|68 BBBB
%def op_sput_wide():
%  op_sput(width="64")


// sput-object vAA, field@BBBB
// Format 21c: AA|69 BBBB
%def op_sput_object():
%  op_sput(width="32", is_object=True)


// sput-object vAA, field@BBBB
// Format 21c: AA|6a BBBB
%def op_sput_boolean():
%  op_sput(width="8")


// sput-object vAA, field@BBBB
// Format 21c: AA|6b BBBB
%def op_sput_byte():
%  op_sput(width="8")


// sput-object vAA, field@BBBB
// Format 21c: AA|6c BBBB
%def op_sput_char():
%  op_sput(width="16")


// sput-object vAA, field@BBBB
// Format 21c: AA|6d BBBB
%def op_sput_short():
%  op_sput(width="16")

