// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: trajectory.proto

#define INTERNAL_SUPPRESS_PROTOBUF_FIELD_DEPRECATION
#include "trajectory.pb.h"

#include <algorithm>

#include <google/protobuf/stubs/common.h>
#include <google/protobuf/stubs/port.h>
#include <google/protobuf/stubs/once.h>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/wire_format_lite_inl.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)

namespace sim_msg {
class TrajectoryPointDefaultTypeInternal : public ::google::protobuf::internal::ExplicitlyConstructed<TrajectoryPoint> {
} _TrajectoryPoint_default_instance_;
class TrajectoryDefaultTypeInternal : public ::google::protobuf::internal::ExplicitlyConstructed<Trajectory> {
} _Trajectory_default_instance_;

namespace protobuf_trajectory_2eproto {


namespace {

::google::protobuf::Metadata file_level_metadata[2];

}  // namespace

const ::google::protobuf::uint32 TableStruct::offsets[] = {
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, x_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, y_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, t_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, v_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, theta_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, kappa_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, s_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, a_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(TrajectoryPoint, z_),
  ~0u,  // no _has_bits_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Trajectory, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Trajectory, point_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Trajectory, a_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Trajectory, type_),
  GOOGLE_PROTOBUF_GENERATED_MESSAGE_FIELD_OFFSET(Trajectory, flag_),
};

static const ::google::protobuf::internal::MigrationSchema schemas[] = {
  { 0, -1, sizeof(TrajectoryPoint)},
  { 13, -1, sizeof(Trajectory)},
};

static ::google::protobuf::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::google::protobuf::Message*>(&_TrajectoryPoint_default_instance_),
  reinterpret_cast<const ::google::protobuf::Message*>(&_Trajectory_default_instance_),
};

namespace {

void protobuf_AssignDescriptors() {
  AddDescriptors();
  ::google::protobuf::MessageFactory* factory = NULL;
  AssignDescriptors(
      "trajectory.proto", schemas, file_default_instances, TableStruct::offsets, factory,
      file_level_metadata, NULL, NULL);
}

void protobuf_AssignDescriptorsOnce() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &protobuf_AssignDescriptors);
}

void protobuf_RegisterTypes(const ::std::string&) GOOGLE_ATTRIBUTE_COLD;
void protobuf_RegisterTypes(const ::std::string&) {
  protobuf_AssignDescriptorsOnce();
  ::google::protobuf::internal::RegisterAllTypes(file_level_metadata, 2);
}

}  // namespace

void TableStruct::Shutdown() {
  _TrajectoryPoint_default_instance_.Shutdown();
  delete file_level_metadata[0].reflection;
  _Trajectory_default_instance_.Shutdown();
  delete file_level_metadata[1].reflection;
}

void TableStruct::InitDefaultsImpl() {
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  ::google::protobuf::internal::InitProtobufDefaults();
  _TrajectoryPoint_default_instance_.DefaultConstruct();
  _Trajectory_default_instance_.DefaultConstruct();
}

void InitDefaults() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &TableStruct::InitDefaultsImpl);
}
void AddDescriptorsImpl() {
  InitDefaults();
  static const char descriptor[] = {
      "\n\020trajectory.proto\022\007sim_msg\"|\n\017Trajector"
      "yPoint\022\t\n\001x\030\001 \001(\001\022\t\n\001y\030\002 \001(\001\022\t\n\001t\030\003 \001(\001\022"
      "\t\n\001v\030\004 \001(\001\022\r\n\005theta\030\005 \001(\001\022\r\n\005kappa\030\006 \001(\001"
      "\022\t\n\001s\030\007 \001(\001\022\t\n\001a\030\010 \001(\001\022\t\n\001z\030\t \001(\001\"\\\n\nTra"
      "jectory\022\'\n\005point\030\001 \003(\0132\030.sim_msg.Traject"
      "oryPoint\022\t\n\001a\030\002 \001(\002\022\014\n\004type\030\003 \001(\005\022\014\n\004fla"
      "g\030\004 \001(\005b\006proto3"
  };
  ::google::protobuf::DescriptorPool::InternalAddGeneratedFile(
      descriptor, 255);
  ::google::protobuf::MessageFactory::InternalRegisterGeneratedFile(
    "trajectory.proto", &protobuf_RegisterTypes);
  ::google::protobuf::internal::OnShutdown(&TableStruct::Shutdown);
}

void AddDescriptors() {
  static GOOGLE_PROTOBUF_DECLARE_ONCE(once);
  ::google::protobuf::GoogleOnceInit(&once, &AddDescriptorsImpl);
}
// Force AddDescriptors() to be called at static initialization time.
struct StaticDescriptorInitializer {
  StaticDescriptorInitializer() {
    AddDescriptors();
  }
} static_descriptor_initializer;

}  // namespace protobuf_trajectory_2eproto


// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int TrajectoryPoint::kXFieldNumber;
const int TrajectoryPoint::kYFieldNumber;
const int TrajectoryPoint::kTFieldNumber;
const int TrajectoryPoint::kVFieldNumber;
const int TrajectoryPoint::kThetaFieldNumber;
const int TrajectoryPoint::kKappaFieldNumber;
const int TrajectoryPoint::kSFieldNumber;
const int TrajectoryPoint::kAFieldNumber;
const int TrajectoryPoint::kZFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

TrajectoryPoint::TrajectoryPoint()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_trajectory_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:sim_msg.TrajectoryPoint)
}
TrajectoryPoint::TrajectoryPoint(const TrajectoryPoint& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&x_, &from.x_,
    reinterpret_cast<char*>(&z_) -
    reinterpret_cast<char*>(&x_) + sizeof(z_));
  // @@protoc_insertion_point(copy_constructor:sim_msg.TrajectoryPoint)
}

void TrajectoryPoint::SharedCtor() {
  ::memset(&x_, 0, reinterpret_cast<char*>(&z_) -
    reinterpret_cast<char*>(&x_) + sizeof(z_));
  _cached_size_ = 0;
}

TrajectoryPoint::~TrajectoryPoint() {
  // @@protoc_insertion_point(destructor:sim_msg.TrajectoryPoint)
  SharedDtor();
}

void TrajectoryPoint::SharedDtor() {
}

void TrajectoryPoint::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* TrajectoryPoint::descriptor() {
  protobuf_trajectory_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_trajectory_2eproto::file_level_metadata[0].descriptor;
}

const TrajectoryPoint& TrajectoryPoint::default_instance() {
  protobuf_trajectory_2eproto::InitDefaults();
  return *internal_default_instance();
}

TrajectoryPoint* TrajectoryPoint::New(::google::protobuf::Arena* arena) const {
  TrajectoryPoint* n = new TrajectoryPoint;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void TrajectoryPoint::Clear() {
// @@protoc_insertion_point(message_clear_start:sim_msg.TrajectoryPoint)
  ::memset(&x_, 0, reinterpret_cast<char*>(&z_) -
    reinterpret_cast<char*>(&x_) + sizeof(z_));
}

bool TrajectoryPoint::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:sim_msg.TrajectoryPoint)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // double x = 1;
      case 1: {
        if (tag == 9u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &x_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double y = 2;
      case 2: {
        if (tag == 17u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &y_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double t = 3;
      case 3: {
        if (tag == 25u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &t_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double v = 4;
      case 4: {
        if (tag == 33u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &v_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double theta = 5;
      case 5: {
        if (tag == 41u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &theta_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double kappa = 6;
      case 6: {
        if (tag == 49u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &kappa_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double s = 7;
      case 7: {
        if (tag == 57u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &s_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double a = 8;
      case 8: {
        if (tag == 65u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &a_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // double z = 9;
      case 9: {
        if (tag == 73u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   double, ::google::protobuf::internal::WireFormatLite::TYPE_DOUBLE>(
                 input, &z_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:sim_msg.TrajectoryPoint)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:sim_msg.TrajectoryPoint)
  return false;
#undef DO_
}

void TrajectoryPoint::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:sim_msg.TrajectoryPoint)
  // double x = 1;
  if (this->x() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(1, this->x(), output);
  }

  // double y = 2;
  if (this->y() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(2, this->y(), output);
  }

  // double t = 3;
  if (this->t() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(3, this->t(), output);
  }

  // double v = 4;
  if (this->v() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(4, this->v(), output);
  }

  // double theta = 5;
  if (this->theta() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(5, this->theta(), output);
  }

  // double kappa = 6;
  if (this->kappa() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(6, this->kappa(), output);
  }

  // double s = 7;
  if (this->s() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(7, this->s(), output);
  }

  // double a = 8;
  if (this->a() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(8, this->a(), output);
  }

  // double z = 9;
  if (this->z() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteDouble(9, this->z(), output);
  }

  // @@protoc_insertion_point(serialize_end:sim_msg.TrajectoryPoint)
}

::google::protobuf::uint8* TrajectoryPoint::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic;  // Unused
  // @@protoc_insertion_point(serialize_to_array_start:sim_msg.TrajectoryPoint)
  // double x = 1;
  if (this->x() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(1, this->x(), target);
  }

  // double y = 2;
  if (this->y() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(2, this->y(), target);
  }

  // double t = 3;
  if (this->t() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(3, this->t(), target);
  }

  // double v = 4;
  if (this->v() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(4, this->v(), target);
  }

  // double theta = 5;
  if (this->theta() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(5, this->theta(), target);
  }

  // double kappa = 6;
  if (this->kappa() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(6, this->kappa(), target);
  }

  // double s = 7;
  if (this->s() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(7, this->s(), target);
  }

  // double a = 8;
  if (this->a() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(8, this->a(), target);
  }

  // double z = 9;
  if (this->z() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteDoubleToArray(9, this->z(), target);
  }

  // @@protoc_insertion_point(serialize_to_array_end:sim_msg.TrajectoryPoint)
  return target;
}

size_t TrajectoryPoint::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:sim_msg.TrajectoryPoint)
  size_t total_size = 0;

  // double x = 1;
  if (this->x() != 0) {
    total_size += 1 + 8;
  }

  // double y = 2;
  if (this->y() != 0) {
    total_size += 1 + 8;
  }

  // double t = 3;
  if (this->t() != 0) {
    total_size += 1 + 8;
  }

  // double v = 4;
  if (this->v() != 0) {
    total_size += 1 + 8;
  }

  // double theta = 5;
  if (this->theta() != 0) {
    total_size += 1 + 8;
  }

  // double kappa = 6;
  if (this->kappa() != 0) {
    total_size += 1 + 8;
  }

  // double s = 7;
  if (this->s() != 0) {
    total_size += 1 + 8;
  }

  // double a = 8;
  if (this->a() != 0) {
    total_size += 1 + 8;
  }

  // double z = 9;
  if (this->z() != 0) {
    total_size += 1 + 8;
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void TrajectoryPoint::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:sim_msg.TrajectoryPoint)
  GOOGLE_DCHECK_NE(&from, this);
  const TrajectoryPoint* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const TrajectoryPoint>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:sim_msg.TrajectoryPoint)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:sim_msg.TrajectoryPoint)
    MergeFrom(*source);
  }
}

void TrajectoryPoint::MergeFrom(const TrajectoryPoint& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:sim_msg.TrajectoryPoint)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  if (from.x() != 0) {
    set_x(from.x());
  }
  if (from.y() != 0) {
    set_y(from.y());
  }
  if (from.t() != 0) {
    set_t(from.t());
  }
  if (from.v() != 0) {
    set_v(from.v());
  }
  if (from.theta() != 0) {
    set_theta(from.theta());
  }
  if (from.kappa() != 0) {
    set_kappa(from.kappa());
  }
  if (from.s() != 0) {
    set_s(from.s());
  }
  if (from.a() != 0) {
    set_a(from.a());
  }
  if (from.z() != 0) {
    set_z(from.z());
  }
}

void TrajectoryPoint::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:sim_msg.TrajectoryPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void TrajectoryPoint::CopyFrom(const TrajectoryPoint& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:sim_msg.TrajectoryPoint)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool TrajectoryPoint::IsInitialized() const {
  return true;
}

void TrajectoryPoint::Swap(TrajectoryPoint* other) {
  if (other == this) return;
  InternalSwap(other);
}
void TrajectoryPoint::InternalSwap(TrajectoryPoint* other) {
  std::swap(x_, other->x_);
  std::swap(y_, other->y_);
  std::swap(t_, other->t_);
  std::swap(v_, other->v_);
  std::swap(theta_, other->theta_);
  std::swap(kappa_, other->kappa_);
  std::swap(s_, other->s_);
  std::swap(a_, other->a_);
  std::swap(z_, other->z_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata TrajectoryPoint::GetMetadata() const {
  protobuf_trajectory_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_trajectory_2eproto::file_level_metadata[0];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// TrajectoryPoint

// double x = 1;
void TrajectoryPoint::clear_x() {
  x_ = 0;
}
double TrajectoryPoint::x() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.x)
  return x_;
}
void TrajectoryPoint::set_x(double value) {
  
  x_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.x)
}

// double y = 2;
void TrajectoryPoint::clear_y() {
  y_ = 0;
}
double TrajectoryPoint::y() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.y)
  return y_;
}
void TrajectoryPoint::set_y(double value) {
  
  y_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.y)
}

// double t = 3;
void TrajectoryPoint::clear_t() {
  t_ = 0;
}
double TrajectoryPoint::t() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.t)
  return t_;
}
void TrajectoryPoint::set_t(double value) {
  
  t_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.t)
}

// double v = 4;
void TrajectoryPoint::clear_v() {
  v_ = 0;
}
double TrajectoryPoint::v() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.v)
  return v_;
}
void TrajectoryPoint::set_v(double value) {
  
  v_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.v)
}

// double theta = 5;
void TrajectoryPoint::clear_theta() {
  theta_ = 0;
}
double TrajectoryPoint::theta() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.theta)
  return theta_;
}
void TrajectoryPoint::set_theta(double value) {
  
  theta_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.theta)
}

// double kappa = 6;
void TrajectoryPoint::clear_kappa() {
  kappa_ = 0;
}
double TrajectoryPoint::kappa() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.kappa)
  return kappa_;
}
void TrajectoryPoint::set_kappa(double value) {
  
  kappa_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.kappa)
}

// double s = 7;
void TrajectoryPoint::clear_s() {
  s_ = 0;
}
double TrajectoryPoint::s() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.s)
  return s_;
}
void TrajectoryPoint::set_s(double value) {
  
  s_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.s)
}

// double a = 8;
void TrajectoryPoint::clear_a() {
  a_ = 0;
}
double TrajectoryPoint::a() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.a)
  return a_;
}
void TrajectoryPoint::set_a(double value) {
  
  a_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.a)
}

// double z = 9;
void TrajectoryPoint::clear_z() {
  z_ = 0;
}
double TrajectoryPoint::z() const {
  // @@protoc_insertion_point(field_get:sim_msg.TrajectoryPoint.z)
  return z_;
}
void TrajectoryPoint::set_z(double value) {
  
  z_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.TrajectoryPoint.z)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// ===================================================================

#if !defined(_MSC_VER) || _MSC_VER >= 1900
const int Trajectory::kPointFieldNumber;
const int Trajectory::kAFieldNumber;
const int Trajectory::kTypeFieldNumber;
const int Trajectory::kFlagFieldNumber;
#endif  // !defined(_MSC_VER) || _MSC_VER >= 1900

Trajectory::Trajectory()
  : ::google::protobuf::Message(), _internal_metadata_(NULL) {
  if (GOOGLE_PREDICT_TRUE(this != internal_default_instance())) {
    protobuf_trajectory_2eproto::InitDefaults();
  }
  SharedCtor();
  // @@protoc_insertion_point(constructor:sim_msg.Trajectory)
}
Trajectory::Trajectory(const Trajectory& from)
  : ::google::protobuf::Message(),
      _internal_metadata_(NULL),
      point_(from.point_),
      _cached_size_(0) {
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  ::memcpy(&a_, &from.a_,
    reinterpret_cast<char*>(&flag_) -
    reinterpret_cast<char*>(&a_) + sizeof(flag_));
  // @@protoc_insertion_point(copy_constructor:sim_msg.Trajectory)
}

void Trajectory::SharedCtor() {
  ::memset(&a_, 0, reinterpret_cast<char*>(&flag_) -
    reinterpret_cast<char*>(&a_) + sizeof(flag_));
  _cached_size_ = 0;
}

Trajectory::~Trajectory() {
  // @@protoc_insertion_point(destructor:sim_msg.Trajectory)
  SharedDtor();
}

void Trajectory::SharedDtor() {
}

void Trajectory::SetCachedSize(int size) const {
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
}
const ::google::protobuf::Descriptor* Trajectory::descriptor() {
  protobuf_trajectory_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_trajectory_2eproto::file_level_metadata[1].descriptor;
}

const Trajectory& Trajectory::default_instance() {
  protobuf_trajectory_2eproto::InitDefaults();
  return *internal_default_instance();
}

Trajectory* Trajectory::New(::google::protobuf::Arena* arena) const {
  Trajectory* n = new Trajectory;
  if (arena != NULL) {
    arena->Own(n);
  }
  return n;
}

void Trajectory::Clear() {
// @@protoc_insertion_point(message_clear_start:sim_msg.Trajectory)
  point_.Clear();
  ::memset(&a_, 0, reinterpret_cast<char*>(&flag_) -
    reinterpret_cast<char*>(&a_) + sizeof(flag_));
}

bool Trajectory::MergePartialFromCodedStream(
    ::google::protobuf::io::CodedInputStream* input) {
#define DO_(EXPRESSION) if (!GOOGLE_PREDICT_TRUE(EXPRESSION)) goto failure
  ::google::protobuf::uint32 tag;
  // @@protoc_insertion_point(parse_start:sim_msg.Trajectory)
  for (;;) {
    ::std::pair< ::google::protobuf::uint32, bool> p = input->ReadTagWithCutoffNoLastTag(127u);
    tag = p.first;
    if (!p.second) goto handle_unusual;
    switch (::google::protobuf::internal::WireFormatLite::GetTagFieldNumber(tag)) {
      // repeated .sim_msg.TrajectoryPoint point = 1;
      case 1: {
        if (tag == 10u) {
          DO_(input->IncrementRecursionDepth());
          DO_(::google::protobuf::internal::WireFormatLite::ReadMessageNoVirtualNoRecursionDepth(
                input, add_point()));
        } else {
          goto handle_unusual;
        }
        input->UnsafeDecrementRecursionDepth();
        break;
      }

      // float a = 2;
      case 2: {
        if (tag == 21u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   float, ::google::protobuf::internal::WireFormatLite::TYPE_FLOAT>(
                 input, &a_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 type = 3;
      case 3: {
        if (tag == 24u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &type_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      // int32 flag = 4;
      case 4: {
        if (tag == 32u) {

          DO_((::google::protobuf::internal::WireFormatLite::ReadPrimitive<
                   ::google::protobuf::int32, ::google::protobuf::internal::WireFormatLite::TYPE_INT32>(
                 input, &flag_)));
        } else {
          goto handle_unusual;
        }
        break;
      }

      default: {
      handle_unusual:
        if (tag == 0 ||
            ::google::protobuf::internal::WireFormatLite::GetTagWireType(tag) ==
            ::google::protobuf::internal::WireFormatLite::WIRETYPE_END_GROUP) {
          goto success;
        }
        DO_(::google::protobuf::internal::WireFormatLite::SkipField(input, tag));
        break;
      }
    }
  }
success:
  // @@protoc_insertion_point(parse_success:sim_msg.Trajectory)
  return true;
failure:
  // @@protoc_insertion_point(parse_failure:sim_msg.Trajectory)
  return false;
#undef DO_
}

void Trajectory::SerializeWithCachedSizes(
    ::google::protobuf::io::CodedOutputStream* output) const {
  // @@protoc_insertion_point(serialize_start:sim_msg.Trajectory)
  // repeated .sim_msg.TrajectoryPoint point = 1;
  for (unsigned int i = 0, n = this->point_size(); i < n; i++) {
    ::google::protobuf::internal::WireFormatLite::WriteMessageMaybeToArray(
      1, this->point(i), output);
  }

  // float a = 2;
  if (this->a() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteFloat(2, this->a(), output);
  }

  // int32 type = 3;
  if (this->type() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(3, this->type(), output);
  }

  // int32 flag = 4;
  if (this->flag() != 0) {
    ::google::protobuf::internal::WireFormatLite::WriteInt32(4, this->flag(), output);
  }

  // @@protoc_insertion_point(serialize_end:sim_msg.Trajectory)
}

::google::protobuf::uint8* Trajectory::InternalSerializeWithCachedSizesToArray(
    bool deterministic, ::google::protobuf::uint8* target) const {
  (void)deterministic;  // Unused
  // @@protoc_insertion_point(serialize_to_array_start:sim_msg.Trajectory)
  // repeated .sim_msg.TrajectoryPoint point = 1;
  for (unsigned int i = 0, n = this->point_size(); i < n; i++) {
    target = ::google::protobuf::internal::WireFormatLite::
      InternalWriteMessageNoVirtualToArray(
        1, this->point(i), false, target);
  }

  // float a = 2;
  if (this->a() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteFloatToArray(2, this->a(), target);
  }

  // int32 type = 3;
  if (this->type() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(3, this->type(), target);
  }

  // int32 flag = 4;
  if (this->flag() != 0) {
    target = ::google::protobuf::internal::WireFormatLite::WriteInt32ToArray(4, this->flag(), target);
  }

  // @@protoc_insertion_point(serialize_to_array_end:sim_msg.Trajectory)
  return target;
}

size_t Trajectory::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:sim_msg.Trajectory)
  size_t total_size = 0;

  // repeated .sim_msg.TrajectoryPoint point = 1;
  {
    unsigned int count = this->point_size();
    total_size += 1UL * count;
    for (unsigned int i = 0; i < count; i++) {
      total_size +=
        ::google::protobuf::internal::WireFormatLite::MessageSizeNoVirtual(
          this->point(i));
    }
  }

  // float a = 2;
  if (this->a() != 0) {
    total_size += 1 + 4;
  }

  // int32 type = 3;
  if (this->type() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->type());
  }

  // int32 flag = 4;
  if (this->flag() != 0) {
    total_size += 1 +
      ::google::protobuf::internal::WireFormatLite::Int32Size(
        this->flag());
  }

  int cached_size = ::google::protobuf::internal::ToCachedSize(total_size);
  GOOGLE_SAFE_CONCURRENT_WRITES_BEGIN();
  _cached_size_ = cached_size;
  GOOGLE_SAFE_CONCURRENT_WRITES_END();
  return total_size;
}

void Trajectory::MergeFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_merge_from_start:sim_msg.Trajectory)
  GOOGLE_DCHECK_NE(&from, this);
  const Trajectory* source =
      ::google::protobuf::internal::DynamicCastToGenerated<const Trajectory>(
          &from);
  if (source == NULL) {
  // @@protoc_insertion_point(generalized_merge_from_cast_fail:sim_msg.Trajectory)
    ::google::protobuf::internal::ReflectionOps::Merge(from, this);
  } else {
  // @@protoc_insertion_point(generalized_merge_from_cast_success:sim_msg.Trajectory)
    MergeFrom(*source);
  }
}

void Trajectory::MergeFrom(const Trajectory& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:sim_msg.Trajectory)
  GOOGLE_DCHECK_NE(&from, this);
  _internal_metadata_.MergeFrom(from._internal_metadata_);
  point_.MergeFrom(from.point_);
  if (from.a() != 0) {
    set_a(from.a());
  }
  if (from.type() != 0) {
    set_type(from.type());
  }
  if (from.flag() != 0) {
    set_flag(from.flag());
  }
}

void Trajectory::CopyFrom(const ::google::protobuf::Message& from) {
// @@protoc_insertion_point(generalized_copy_from_start:sim_msg.Trajectory)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

void Trajectory::CopyFrom(const Trajectory& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:sim_msg.Trajectory)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Trajectory::IsInitialized() const {
  return true;
}

void Trajectory::Swap(Trajectory* other) {
  if (other == this) return;
  InternalSwap(other);
}
void Trajectory::InternalSwap(Trajectory* other) {
  point_.UnsafeArenaSwap(&other->point_);
  std::swap(a_, other->a_);
  std::swap(type_, other->type_);
  std::swap(flag_, other->flag_);
  std::swap(_cached_size_, other->_cached_size_);
}

::google::protobuf::Metadata Trajectory::GetMetadata() const {
  protobuf_trajectory_2eproto::protobuf_AssignDescriptorsOnce();
  return protobuf_trajectory_2eproto::file_level_metadata[1];
}

#if PROTOBUF_INLINE_NOT_IN_HEADERS
// Trajectory

// repeated .sim_msg.TrajectoryPoint point = 1;
int Trajectory::point_size() const {
  return point_.size();
}
void Trajectory::clear_point() {
  point_.Clear();
}
const ::sim_msg::TrajectoryPoint& Trajectory::point(int index) const {
  // @@protoc_insertion_point(field_get:sim_msg.Trajectory.point)
  return point_.Get(index);
}
::sim_msg::TrajectoryPoint* Trajectory::mutable_point(int index) {
  // @@protoc_insertion_point(field_mutable:sim_msg.Trajectory.point)
  return point_.Mutable(index);
}
::sim_msg::TrajectoryPoint* Trajectory::add_point() {
  // @@protoc_insertion_point(field_add:sim_msg.Trajectory.point)
  return point_.Add();
}
::google::protobuf::RepeatedPtrField< ::sim_msg::TrajectoryPoint >*
Trajectory::mutable_point() {
  // @@protoc_insertion_point(field_mutable_list:sim_msg.Trajectory.point)
  return &point_;
}
const ::google::protobuf::RepeatedPtrField< ::sim_msg::TrajectoryPoint >&
Trajectory::point() const {
  // @@protoc_insertion_point(field_list:sim_msg.Trajectory.point)
  return point_;
}

// float a = 2;
void Trajectory::clear_a() {
  a_ = 0;
}
float Trajectory::a() const {
  // @@protoc_insertion_point(field_get:sim_msg.Trajectory.a)
  return a_;
}
void Trajectory::set_a(float value) {
  
  a_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.Trajectory.a)
}

// int32 type = 3;
void Trajectory::clear_type() {
  type_ = 0;
}
::google::protobuf::int32 Trajectory::type() const {
  // @@protoc_insertion_point(field_get:sim_msg.Trajectory.type)
  return type_;
}
void Trajectory::set_type(::google::protobuf::int32 value) {
  
  type_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.Trajectory.type)
}

// int32 flag = 4;
void Trajectory::clear_flag() {
  flag_ = 0;
}
::google::protobuf::int32 Trajectory::flag() const {
  // @@protoc_insertion_point(field_get:sim_msg.Trajectory.flag)
  return flag_;
}
void Trajectory::set_flag(::google::protobuf::int32 value) {
  
  flag_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.Trajectory.flag)
}

#endif  // PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)

}  // namespace sim_msg

// @@protoc_insertion_point(global_scope)
