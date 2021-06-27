// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: basic.proto

#include "basic.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace sim_msg {
constexpr Vec2::Vec2(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : x_(0)
  , y_(0){}
struct Vec2DefaultTypeInternal {
  constexpr Vec2DefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~Vec2DefaultTypeInternal() {}
  union {
    Vec2 _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT Vec2DefaultTypeInternal _Vec2_default_instance_;
constexpr Vec3::Vec3(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : x_(0)
  , y_(0)
  , z_(0){}
struct Vec3DefaultTypeInternal {
  constexpr Vec3DefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~Vec3DefaultTypeInternal() {}
  union {
    Vec3 _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT Vec3DefaultTypeInternal _Vec3_default_instance_;
constexpr Vec4::Vec4(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : x_(0)
  , y_(0)
  , z_(0)
  , w_(0){}
struct Vec4DefaultTypeInternal {
  constexpr Vec4DefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~Vec4DefaultTypeInternal() {}
  union {
    Vec4 _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT Vec4DefaultTypeInternal _Vec4_default_instance_;
}  // namespace sim_msg
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_basic_2eproto[3];
static constexpr ::PROTOBUF_NAMESPACE_ID::EnumDescriptor const** file_level_enum_descriptors_basic_2eproto = nullptr;
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_basic_2eproto = nullptr;

const ::PROTOBUF_NAMESPACE_ID::uint32 TableStruct_basic_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec2, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec2, x_),
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec2, y_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec3, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec3, x_),
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec3, y_),
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec3, z_),
  ~0u,  // no _has_bits_
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec4, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec4, x_),
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec4, y_),
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec4, z_),
  PROTOBUF_FIELD_OFFSET(::sim_msg::Vec4, w_),
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, -1, sizeof(::sim_msg::Vec2)},
  { 7, -1, sizeof(::sim_msg::Vec3)},
  { 15, -1, sizeof(::sim_msg::Vec4)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::sim_msg::_Vec2_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::sim_msg::_Vec3_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::sim_msg::_Vec4_default_instance_),
};

const char descriptor_table_protodef_basic_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\013basic.proto\022\007sim_msg\"\034\n\004Vec2\022\t\n\001x\030\001 \001("
  "\001\022\t\n\001y\030\002 \001(\001\"\'\n\004Vec3\022\t\n\001x\030\001 \001(\001\022\t\n\001y\030\002 \001"
  "(\001\022\t\n\001z\030\003 \001(\001\"2\n\004Vec4\022\t\n\001x\030\001 \001(\001\022\t\n\001y\030\002 "
  "\001(\001\022\t\n\001z\030\003 \001(\001\022\t\n\001w\030\004 \001(\001b\006proto3"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_basic_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_basic_2eproto = {
  false, false, 153, descriptor_table_protodef_basic_2eproto, "basic.proto", 
  &descriptor_table_basic_2eproto_once, nullptr, 0, 3,
  schemas, file_default_instances, TableStruct_basic_2eproto::offsets,
  file_level_metadata_basic_2eproto, file_level_enum_descriptors_basic_2eproto, file_level_service_descriptors_basic_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_basic_2eproto_getter() {
  return &descriptor_table_basic_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_basic_2eproto(&descriptor_table_basic_2eproto);
namespace sim_msg {

// ===================================================================

class Vec2::_Internal {
 public:
};

Vec2::Vec2(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:sim_msg.Vec2)
}
Vec2::Vec2(const Vec2& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&y_) -
    reinterpret_cast<char*>(&x_)) + sizeof(y_));
  // @@protoc_insertion_point(copy_constructor:sim_msg.Vec2)
}

inline void Vec2::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&x_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&y_) -
    reinterpret_cast<char*>(&x_)) + sizeof(y_));
}

Vec2::~Vec2() {
  // @@protoc_insertion_point(destructor:sim_msg.Vec2)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Vec2::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Vec2::ArenaDtor(void* object) {
  Vec2* _this = reinterpret_cast< Vec2* >(object);
  (void)_this;
}
void Vec2::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Vec2::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Vec2::Clear() {
// @@protoc_insertion_point(message_clear_start:sim_msg.Vec2)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&y_) -
      reinterpret_cast<char*>(&x_)) + sizeof(y_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Vec2::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // double x = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double y = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag == 0) || ((tag & 7) == 4)) {
          CHK_(ptr);
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Vec2::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:sim_msg.Vec2)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double x = 1;
  if (!(this->_internal_x() <= 0 && this->_internal_x() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_x(), target);
  }

  // double y = 2;
  if (!(this->_internal_y() <= 0 && this->_internal_y() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_y(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:sim_msg.Vec2)
  return target;
}

size_t Vec2::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:sim_msg.Vec2)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double x = 1;
  if (!(this->_internal_x() <= 0 && this->_internal_x() >= 0)) {
    total_size += 1 + 8;
  }

  // double y = 2;
  if (!(this->_internal_y() <= 0 && this->_internal_y() >= 0)) {
    total_size += 1 + 8;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Vec2::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Vec2::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Vec2::GetClassData() const { return &_class_data_; }

void Vec2::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to,
                      const ::PROTOBUF_NAMESPACE_ID::Message&from) {
  static_cast<Vec2 *>(to)->MergeFrom(
      static_cast<const Vec2 &>(from));
}


void Vec2::MergeFrom(const Vec2& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:sim_msg.Vec2)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (!(from._internal_x() <= 0 && from._internal_x() >= 0)) {
    _internal_set_x(from._internal_x());
  }
  if (!(from._internal_y() <= 0 && from._internal_y() >= 0)) {
    _internal_set_y(from._internal_y());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Vec2::CopyFrom(const Vec2& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:sim_msg.Vec2)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vec2::IsInitialized() const {
  return true;
}

void Vec2::InternalSwap(Vec2* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Vec2, y_)
      + sizeof(Vec2::y_)
      - PROTOBUF_FIELD_OFFSET(Vec2, x_)>(
          reinterpret_cast<char*>(&x_),
          reinterpret_cast<char*>(&other->x_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Vec2::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_basic_2eproto_getter, &descriptor_table_basic_2eproto_once,
      file_level_metadata_basic_2eproto[0]);
}

// ===================================================================

class Vec3::_Internal {
 public:
};

Vec3::Vec3(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:sim_msg.Vec3)
}
Vec3::Vec3(const Vec3& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&z_) -
    reinterpret_cast<char*>(&x_)) + sizeof(z_));
  // @@protoc_insertion_point(copy_constructor:sim_msg.Vec3)
}

inline void Vec3::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&x_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&z_) -
    reinterpret_cast<char*>(&x_)) + sizeof(z_));
}

Vec3::~Vec3() {
  // @@protoc_insertion_point(destructor:sim_msg.Vec3)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Vec3::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Vec3::ArenaDtor(void* object) {
  Vec3* _this = reinterpret_cast< Vec3* >(object);
  (void)_this;
}
void Vec3::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Vec3::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Vec3::Clear() {
// @@protoc_insertion_point(message_clear_start:sim_msg.Vec3)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&z_) -
      reinterpret_cast<char*>(&x_)) + sizeof(z_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Vec3::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // double x = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double y = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double z = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          z_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag == 0) || ((tag & 7) == 4)) {
          CHK_(ptr);
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Vec3::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:sim_msg.Vec3)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double x = 1;
  if (!(this->_internal_x() <= 0 && this->_internal_x() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_x(), target);
  }

  // double y = 2;
  if (!(this->_internal_y() <= 0 && this->_internal_y() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_y(), target);
  }

  // double z = 3;
  if (!(this->_internal_z() <= 0 && this->_internal_z() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_z(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:sim_msg.Vec3)
  return target;
}

size_t Vec3::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:sim_msg.Vec3)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double x = 1;
  if (!(this->_internal_x() <= 0 && this->_internal_x() >= 0)) {
    total_size += 1 + 8;
  }

  // double y = 2;
  if (!(this->_internal_y() <= 0 && this->_internal_y() >= 0)) {
    total_size += 1 + 8;
  }

  // double z = 3;
  if (!(this->_internal_z() <= 0 && this->_internal_z() >= 0)) {
    total_size += 1 + 8;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Vec3::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Vec3::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Vec3::GetClassData() const { return &_class_data_; }

void Vec3::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to,
                      const ::PROTOBUF_NAMESPACE_ID::Message&from) {
  static_cast<Vec3 *>(to)->MergeFrom(
      static_cast<const Vec3 &>(from));
}


void Vec3::MergeFrom(const Vec3& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:sim_msg.Vec3)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (!(from._internal_x() <= 0 && from._internal_x() >= 0)) {
    _internal_set_x(from._internal_x());
  }
  if (!(from._internal_y() <= 0 && from._internal_y() >= 0)) {
    _internal_set_y(from._internal_y());
  }
  if (!(from._internal_z() <= 0 && from._internal_z() >= 0)) {
    _internal_set_z(from._internal_z());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Vec3::CopyFrom(const Vec3& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:sim_msg.Vec3)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vec3::IsInitialized() const {
  return true;
}

void Vec3::InternalSwap(Vec3* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Vec3, z_)
      + sizeof(Vec3::z_)
      - PROTOBUF_FIELD_OFFSET(Vec3, x_)>(
          reinterpret_cast<char*>(&x_),
          reinterpret_cast<char*>(&other->x_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Vec3::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_basic_2eproto_getter, &descriptor_table_basic_2eproto_once,
      file_level_metadata_basic_2eproto[1]);
}

// ===================================================================

class Vec4::_Internal {
 public:
};

Vec4::Vec4(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:sim_msg.Vec4)
}
Vec4::Vec4(const Vec4& from)
  : ::PROTOBUF_NAMESPACE_ID::Message() {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    static_cast<size_t>(reinterpret_cast<char*>(&w_) -
    reinterpret_cast<char*>(&x_)) + sizeof(w_));
  // @@protoc_insertion_point(copy_constructor:sim_msg.Vec4)
}

inline void Vec4::SharedCtor() {
::memset(reinterpret_cast<char*>(this) + static_cast<size_t>(
    reinterpret_cast<char*>(&x_) - reinterpret_cast<char*>(this)),
    0, static_cast<size_t>(reinterpret_cast<char*>(&w_) -
    reinterpret_cast<char*>(&x_)) + sizeof(w_));
}

Vec4::~Vec4() {
  // @@protoc_insertion_point(destructor:sim_msg.Vec4)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Vec4::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
}

void Vec4::ArenaDtor(void* object) {
  Vec4* _this = reinterpret_cast< Vec4* >(object);
  (void)_this;
}
void Vec4::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Vec4::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Vec4::Clear() {
// @@protoc_insertion_point(message_clear_start:sim_msg.Vec4)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  ::memset(&x_, 0, static_cast<size_t>(
      reinterpret_cast<char*>(&w_) -
      reinterpret_cast<char*>(&x_)) + sizeof(w_));
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Vec4::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  while (!ctx->Done(&ptr)) {
    ::PROTOBUF_NAMESPACE_ID::uint32 tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // double x = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 9)) {
          x_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double y = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 17)) {
          y_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double z = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 25)) {
          z_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      // double w = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<::PROTOBUF_NAMESPACE_ID::uint8>(tag) == 33)) {
          w_ = ::PROTOBUF_NAMESPACE_ID::internal::UnalignedLoad<double>(ptr);
          ptr += sizeof(double);
        } else goto handle_unusual;
        continue;
      default: {
      handle_unusual:
        if ((tag == 0) || ((tag & 7) == 4)) {
          CHK_(ptr);
          ctx->SetLastTag(tag);
          goto success;
        }
        ptr = UnknownFieldParse(tag,
            _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
            ptr, ctx);
        CHK_(ptr != nullptr);
        continue;
      }
    }  // switch
  }  // while
success:
  return ptr;
failure:
  ptr = nullptr;
  goto success;
#undef CHK_
}

::PROTOBUF_NAMESPACE_ID::uint8* Vec4::_InternalSerialize(
    ::PROTOBUF_NAMESPACE_ID::uint8* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:sim_msg.Vec4)
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  // double x = 1;
  if (!(this->_internal_x() <= 0 && this->_internal_x() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(1, this->_internal_x(), target);
  }

  // double y = 2;
  if (!(this->_internal_y() <= 0 && this->_internal_y() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(2, this->_internal_y(), target);
  }

  // double z = 3;
  if (!(this->_internal_z() <= 0 && this->_internal_z() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(3, this->_internal_z(), target);
  }

  // double w = 4;
  if (!(this->_internal_w() <= 0 && this->_internal_w() >= 0)) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteDoubleToArray(4, this->_internal_w(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:sim_msg.Vec4)
  return target;
}

size_t Vec4::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:sim_msg.Vec4)
  size_t total_size = 0;

  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // double x = 1;
  if (!(this->_internal_x() <= 0 && this->_internal_x() >= 0)) {
    total_size += 1 + 8;
  }

  // double y = 2;
  if (!(this->_internal_y() <= 0 && this->_internal_y() >= 0)) {
    total_size += 1 + 8;
  }

  // double z = 3;
  if (!(this->_internal_z() <= 0 && this->_internal_z() >= 0)) {
    total_size += 1 + 8;
  }

  // double w = 4;
  if (!(this->_internal_w() <= 0 && this->_internal_w() >= 0)) {
    total_size += 1 + 8;
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    return ::PROTOBUF_NAMESPACE_ID::internal::ComputeUnknownFieldsSize(
        _internal_metadata_, total_size, &_cached_size_);
  }
  int cached_size = ::PROTOBUF_NAMESPACE_ID::internal::ToCachedSize(total_size);
  SetCachedSize(cached_size);
  return total_size;
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Vec4::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Vec4::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Vec4::GetClassData() const { return &_class_data_; }

void Vec4::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message*to,
                      const ::PROTOBUF_NAMESPACE_ID::Message&from) {
  static_cast<Vec4 *>(to)->MergeFrom(
      static_cast<const Vec4 &>(from));
}


void Vec4::MergeFrom(const Vec4& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:sim_msg.Vec4)
  GOOGLE_DCHECK_NE(&from, this);
  ::PROTOBUF_NAMESPACE_ID::uint32 cached_has_bits = 0;
  (void) cached_has_bits;

  if (!(from._internal_x() <= 0 && from._internal_x() >= 0)) {
    _internal_set_x(from._internal_x());
  }
  if (!(from._internal_y() <= 0 && from._internal_y() >= 0)) {
    _internal_set_y(from._internal_y());
  }
  if (!(from._internal_z() <= 0 && from._internal_z() >= 0)) {
    _internal_set_z(from._internal_z());
  }
  if (!(from._internal_w() <= 0 && from._internal_w() >= 0)) {
    _internal_set_w(from._internal_w());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Vec4::CopyFrom(const Vec4& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:sim_msg.Vec4)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Vec4::IsInitialized() const {
  return true;
}

void Vec4::InternalSwap(Vec4* other) {
  using std::swap;
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  ::PROTOBUF_NAMESPACE_ID::internal::memswap<
      PROTOBUF_FIELD_OFFSET(Vec4, w_)
      + sizeof(Vec4::w_)
      - PROTOBUF_FIELD_OFFSET(Vec4, x_)>(
          reinterpret_cast<char*>(&x_),
          reinterpret_cast<char*>(&other->x_));
}

::PROTOBUF_NAMESPACE_ID::Metadata Vec4::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_basic_2eproto_getter, &descriptor_table_basic_2eproto_once,
      file_level_metadata_basic_2eproto[2]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace sim_msg
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::sim_msg::Vec2* Arena::CreateMaybeMessage< ::sim_msg::Vec2 >(Arena* arena) {
  return Arena::CreateMessageInternal< ::sim_msg::Vec2 >(arena);
}
template<> PROTOBUF_NOINLINE ::sim_msg::Vec3* Arena::CreateMaybeMessage< ::sim_msg::Vec3 >(Arena* arena) {
  return Arena::CreateMessageInternal< ::sim_msg::Vec3 >(arena);
}
template<> PROTOBUF_NOINLINE ::sim_msg::Vec4* Arena::CreateMaybeMessage< ::sim_msg::Vec4 >(Arena* arena) {
  return Arena::CreateMessageInternal< ::sim_msg::Vec4 >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
