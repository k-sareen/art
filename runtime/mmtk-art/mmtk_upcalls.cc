#include "mmtk_upcalls.h"
#include "mirror/object-inl.h"

REQUIRES_SHARED(art::Locks::mutator_lock_)
static size_t size_of(void *object) {
  art::mirror::Object *obj = (art::mirror::Object *) object;
  return obj->SizeOf();
}

ArtUpcalls art_upcalls = {
  size_of,
};
