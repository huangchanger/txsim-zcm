// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: planStatus.proto

#ifndef PROTOBUF_planStatus_2eproto__INCLUDED
#define PROTOBUF_planStatus_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 3002000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 3002000 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace sim_msg {
class AccelerationExpectation;
class AccelerationExpectationDefaultTypeInternal;
extern AccelerationExpectationDefaultTypeInternal _AccelerationExpectation_default_instance_;
class PlanStatus;
class PlanStatusDefaultTypeInternal;
extern PlanStatusDefaultTypeInternal _PlanStatus_default_instance_;
class SteeringExpectation;
class SteeringExpectationDefaultTypeInternal;
extern SteeringExpectationDefaultTypeInternal _SteeringExpectation_default_instance_;
}  // namespace sim_msg

namespace sim_msg {

namespace protobuf_planStatus_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::uint32 offsets[];
  static void InitDefaultsImpl();
  static void Shutdown();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_planStatus_2eproto

enum IndicatorState {
  INDICATOR_STATE_UNKNOWN = 0,
  INDICATOR_STATE_OFF = 1,
  INDICATOR_STATE_LEFT = 2,
  INDICATOR_STATE_RIGHT = 3,
  INDICATOR_STATE_WARNING = 4,
  IndicatorState_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  IndicatorState_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool IndicatorState_IsValid(int value);
const IndicatorState IndicatorState_MIN = INDICATOR_STATE_UNKNOWN;
const IndicatorState IndicatorState_MAX = INDICATOR_STATE_WARNING;
const int IndicatorState_ARRAYSIZE = IndicatorState_MAX + 1;

const ::google::protobuf::EnumDescriptor* IndicatorState_descriptor();
inline const ::std::string& IndicatorState_Name(IndicatorState value) {
  return ::google::protobuf::internal::NameOfEnum(
    IndicatorState_descriptor(), value);
}
inline bool IndicatorState_Parse(
    const ::std::string& name, IndicatorState* value) {
  return ::google::protobuf::internal::ParseNamedEnum<IndicatorState>(
    IndicatorState_descriptor(), name, value);
}
enum PlanMode {
  PLAN_MODE_FOLLOW_LANE = 0,
  PLAN_MODE_CHANGE_LANE = 1,
  PLAN_MODE_MANUAL = 2,
  PlanMode_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  PlanMode_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool PlanMode_IsValid(int value);
const PlanMode PlanMode_MIN = PLAN_MODE_FOLLOW_LANE;
const PlanMode PlanMode_MAX = PLAN_MODE_MANUAL;
const int PlanMode_ARRAYSIZE = PlanMode_MAX + 1;

const ::google::protobuf::EnumDescriptor* PlanMode_descriptor();
inline const ::std::string& PlanMode_Name(PlanMode value) {
  return ::google::protobuf::internal::NameOfEnum(
    PlanMode_descriptor(), value);
}
inline bool PlanMode_Parse(
    const ::std::string& name, PlanMode* value) {
  return ::google::protobuf::internal::ParseNamedEnum<PlanMode>(
    PlanMode_descriptor(), name, value);
}
enum PlanClassification {
  PLAN_GO_STRAIGHT = 0,
  PLAN_TURN_LEFT = 1,
  PLAN_TURN_RIGHT = 2,
  PLAN_CHANGE_LANE_LEFT = 3,
  PLAN_CHANGE_LANE_RIGHT = 4,
  PLAN_PULL_OVER = 5,
  PLAN_TURN_AROUND = 6,
  PlanClassification_INT_MIN_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32min,
  PlanClassification_INT_MAX_SENTINEL_DO_NOT_USE_ = ::google::protobuf::kint32max
};
bool PlanClassification_IsValid(int value);
const PlanClassification PlanClassification_MIN = PLAN_GO_STRAIGHT;
const PlanClassification PlanClassification_MAX = PLAN_TURN_AROUND;
const int PlanClassification_ARRAYSIZE = PlanClassification_MAX + 1;

const ::google::protobuf::EnumDescriptor* PlanClassification_descriptor();
inline const ::std::string& PlanClassification_Name(PlanClassification value) {
  return ::google::protobuf::internal::NameOfEnum(
    PlanClassification_descriptor(), value);
}
inline bool PlanClassification_Parse(
    const ::std::string& name, PlanClassification* value) {
  return ::google::protobuf::internal::ParseNamedEnum<PlanClassification>(
    PlanClassification_descriptor(), name, value);
}
// ===================================================================

class SteeringExpectation : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:sim_msg.SteeringExpectation) */ {
 public:
  SteeringExpectation();
  virtual ~SteeringExpectation();

  SteeringExpectation(const SteeringExpectation& from);

  inline SteeringExpectation& operator=(const SteeringExpectation& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const SteeringExpectation& default_instance();

  static inline const SteeringExpectation* internal_default_instance() {
    return reinterpret_cast<const SteeringExpectation*>(
               &_SteeringExpectation_default_instance_);
  }

  void Swap(SteeringExpectation* other);

  // implements Message ----------------------------------------------

  inline SteeringExpectation* New() const PROTOBUF_FINAL { return New(NULL); }

  SteeringExpectation* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const SteeringExpectation& from);
  void MergeFrom(const SteeringExpectation& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output)
      const PROTOBUF_FINAL {
    return InternalSerializeWithCachedSizesToArray(
        ::google::protobuf::io::CodedOutputStream::IsDefaultSerializationDeterministic(), output);
  }
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(SteeringExpectation* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // double angle = 1;
  void clear_angle();
  static const int kAngleFieldNumber = 1;
  double angle() const;
  void set_angle(double value);

  // @@protoc_insertion_point(class_scope:sim_msg.SteeringExpectation)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double angle_;
  mutable int _cached_size_;
  friend struct  protobuf_planStatus_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class AccelerationExpectation : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:sim_msg.AccelerationExpectation) */ {
 public:
  AccelerationExpectation();
  virtual ~AccelerationExpectation();

  AccelerationExpectation(const AccelerationExpectation& from);

  inline AccelerationExpectation& operator=(const AccelerationExpectation& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const AccelerationExpectation& default_instance();

  static inline const AccelerationExpectation* internal_default_instance() {
    return reinterpret_cast<const AccelerationExpectation*>(
               &_AccelerationExpectation_default_instance_);
  }

  void Swap(AccelerationExpectation* other);

  // implements Message ----------------------------------------------

  inline AccelerationExpectation* New() const PROTOBUF_FINAL { return New(NULL); }

  AccelerationExpectation* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const AccelerationExpectation& from);
  void MergeFrom(const AccelerationExpectation& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output)
      const PROTOBUF_FINAL {
    return InternalSerializeWithCachedSizesToArray(
        ::google::protobuf::io::CodedOutputStream::IsDefaultSerializationDeterministic(), output);
  }
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(AccelerationExpectation* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // double acc = 1;
  void clear_acc();
  static const int kAccFieldNumber = 1;
  double acc() const;
  void set_acc(double value);

  // @@protoc_insertion_point(class_scope:sim_msg.AccelerationExpectation)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double acc_;
  mutable int _cached_size_;
  friend struct  protobuf_planStatus_2eproto::TableStruct;
};
// -------------------------------------------------------------------

class PlanStatus : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:sim_msg.PlanStatus) */ {
 public:
  PlanStatus();
  virtual ~PlanStatus();

  PlanStatus(const PlanStatus& from);

  inline PlanStatus& operator=(const PlanStatus& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const PlanStatus& default_instance();

  static inline const PlanStatus* internal_default_instance() {
    return reinterpret_cast<const PlanStatus*>(
               &_PlanStatus_default_instance_);
  }

  void Swap(PlanStatus* other);

  // implements Message ----------------------------------------------

  inline PlanStatus* New() const PROTOBUF_FINAL { return New(NULL); }

  PlanStatus* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const PlanStatus& from);
  void MergeFrom(const PlanStatus& from);
  void Clear() PROTOBUF_FINAL;
  bool IsInitialized() const PROTOBUF_FINAL;

  size_t ByteSizeLong() const PROTOBUF_FINAL;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input) PROTOBUF_FINAL;
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* InternalSerializeWithCachedSizesToArray(
      bool deterministic, ::google::protobuf::uint8* target) const PROTOBUF_FINAL;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output)
      const PROTOBUF_FINAL {
    return InternalSerializeWithCachedSizesToArray(
        ::google::protobuf::io::CodedOutputStream::IsDefaultSerializationDeterministic(), output);
  }
  int GetCachedSize() const PROTOBUF_FINAL { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const PROTOBUF_FINAL;
  void InternalSwap(PlanStatus* other);
  private:
  inline ::google::protobuf::Arena* GetArenaNoVirtual() const {
    return NULL;
  }
  inline void* MaybeArenaPtr() const {
    return NULL;
  }
  public:

  ::google::protobuf::Metadata GetMetadata() const PROTOBUF_FINAL;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // .sim_msg.SteeringExpectation expect_steering = 4;
  bool has_expect_steering() const;
  void clear_expect_steering();
  static const int kExpectSteeringFieldNumber = 4;
  const ::sim_msg::SteeringExpectation& expect_steering() const;
  ::sim_msg::SteeringExpectation* mutable_expect_steering();
  ::sim_msg::SteeringExpectation* release_expect_steering();
  void set_allocated_expect_steering(::sim_msg::SteeringExpectation* expect_steering);

  // .sim_msg.AccelerationExpectation expect_acc = 5;
  bool has_expect_acc() const;
  void clear_expect_acc();
  static const int kExpectAccFieldNumber = 5;
  const ::sim_msg::AccelerationExpectation& expect_acc() const;
  ::sim_msg::AccelerationExpectation* mutable_expect_acc();
  ::sim_msg::AccelerationExpectation* release_expect_acc();
  void set_allocated_expect_acc(::sim_msg::AccelerationExpectation* expect_acc);

  // .sim_msg.IndicatorState indicator_state = 1;
  void clear_indicator_state();
  static const int kIndicatorStateFieldNumber = 1;
  ::sim_msg::IndicatorState indicator_state() const;
  void set_indicator_state(::sim_msg::IndicatorState value);

  // .sim_msg.PlanMode mode = 2;
  void clear_mode();
  static const int kModeFieldNumber = 2;
  ::sim_msg::PlanMode mode() const;
  void set_mode(::sim_msg::PlanMode value);

  // .sim_msg.PlanClassification class = 3;
  void clear_class_();
  static const int kClassFieldNumber = 3;
  ::sim_msg::PlanClassification class_() const;
  void set_class_(::sim_msg::PlanClassification value);

  // @@protoc_insertion_point(class_scope:sim_msg.PlanStatus)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  ::sim_msg::SteeringExpectation* expect_steering_;
  ::sim_msg::AccelerationExpectation* expect_acc_;
  int indicator_state_;
  int mode_;
  int class__;
  mutable int _cached_size_;
  friend struct  protobuf_planStatus_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// SteeringExpectation

// double angle = 1;
inline void SteeringExpectation::clear_angle() {
  angle_ = 0;
}
inline double SteeringExpectation::angle() const {
  // @@protoc_insertion_point(field_get:sim_msg.SteeringExpectation.angle)
  return angle_;
}
inline void SteeringExpectation::set_angle(double value) {
  
  angle_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.SteeringExpectation.angle)
}

// -------------------------------------------------------------------

// AccelerationExpectation

// double acc = 1;
inline void AccelerationExpectation::clear_acc() {
  acc_ = 0;
}
inline double AccelerationExpectation::acc() const {
  // @@protoc_insertion_point(field_get:sim_msg.AccelerationExpectation.acc)
  return acc_;
}
inline void AccelerationExpectation::set_acc(double value) {
  
  acc_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.AccelerationExpectation.acc)
}

// -------------------------------------------------------------------

// PlanStatus

// .sim_msg.IndicatorState indicator_state = 1;
inline void PlanStatus::clear_indicator_state() {
  indicator_state_ = 0;
}
inline ::sim_msg::IndicatorState PlanStatus::indicator_state() const {
  // @@protoc_insertion_point(field_get:sim_msg.PlanStatus.indicator_state)
  return static_cast< ::sim_msg::IndicatorState >(indicator_state_);
}
inline void PlanStatus::set_indicator_state(::sim_msg::IndicatorState value) {
  
  indicator_state_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.PlanStatus.indicator_state)
}

// .sim_msg.PlanMode mode = 2;
inline void PlanStatus::clear_mode() {
  mode_ = 0;
}
inline ::sim_msg::PlanMode PlanStatus::mode() const {
  // @@protoc_insertion_point(field_get:sim_msg.PlanStatus.mode)
  return static_cast< ::sim_msg::PlanMode >(mode_);
}
inline void PlanStatus::set_mode(::sim_msg::PlanMode value) {
  
  mode_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.PlanStatus.mode)
}

// .sim_msg.PlanClassification class = 3;
inline void PlanStatus::clear_class_() {
  class__ = 0;
}
inline ::sim_msg::PlanClassification PlanStatus::class_() const {
  // @@protoc_insertion_point(field_get:sim_msg.PlanStatus.class)
  return static_cast< ::sim_msg::PlanClassification >(class__);
}
inline void PlanStatus::set_class_(::sim_msg::PlanClassification value) {
  
  class__ = value;
  // @@protoc_insertion_point(field_set:sim_msg.PlanStatus.class)
}

// .sim_msg.SteeringExpectation expect_steering = 4;
inline bool PlanStatus::has_expect_steering() const {
  return this != internal_default_instance() && expect_steering_ != NULL;
}
inline void PlanStatus::clear_expect_steering() {
  if (GetArenaNoVirtual() == NULL && expect_steering_ != NULL) delete expect_steering_;
  expect_steering_ = NULL;
}
inline const ::sim_msg::SteeringExpectation& PlanStatus::expect_steering() const {
  // @@protoc_insertion_point(field_get:sim_msg.PlanStatus.expect_steering)
  return expect_steering_ != NULL ? *expect_steering_
                         : *::sim_msg::SteeringExpectation::internal_default_instance();
}
inline ::sim_msg::SteeringExpectation* PlanStatus::mutable_expect_steering() {
  
  if (expect_steering_ == NULL) {
    expect_steering_ = new ::sim_msg::SteeringExpectation;
  }
  // @@protoc_insertion_point(field_mutable:sim_msg.PlanStatus.expect_steering)
  return expect_steering_;
}
inline ::sim_msg::SteeringExpectation* PlanStatus::release_expect_steering() {
  // @@protoc_insertion_point(field_release:sim_msg.PlanStatus.expect_steering)
  
  ::sim_msg::SteeringExpectation* temp = expect_steering_;
  expect_steering_ = NULL;
  return temp;
}
inline void PlanStatus::set_allocated_expect_steering(::sim_msg::SteeringExpectation* expect_steering) {
  delete expect_steering_;
  expect_steering_ = expect_steering;
  if (expect_steering) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:sim_msg.PlanStatus.expect_steering)
}

// .sim_msg.AccelerationExpectation expect_acc = 5;
inline bool PlanStatus::has_expect_acc() const {
  return this != internal_default_instance() && expect_acc_ != NULL;
}
inline void PlanStatus::clear_expect_acc() {
  if (GetArenaNoVirtual() == NULL && expect_acc_ != NULL) delete expect_acc_;
  expect_acc_ = NULL;
}
inline const ::sim_msg::AccelerationExpectation& PlanStatus::expect_acc() const {
  // @@protoc_insertion_point(field_get:sim_msg.PlanStatus.expect_acc)
  return expect_acc_ != NULL ? *expect_acc_
                         : *::sim_msg::AccelerationExpectation::internal_default_instance();
}
inline ::sim_msg::AccelerationExpectation* PlanStatus::mutable_expect_acc() {
  
  if (expect_acc_ == NULL) {
    expect_acc_ = new ::sim_msg::AccelerationExpectation;
  }
  // @@protoc_insertion_point(field_mutable:sim_msg.PlanStatus.expect_acc)
  return expect_acc_;
}
inline ::sim_msg::AccelerationExpectation* PlanStatus::release_expect_acc() {
  // @@protoc_insertion_point(field_release:sim_msg.PlanStatus.expect_acc)
  
  ::sim_msg::AccelerationExpectation* temp = expect_acc_;
  expect_acc_ = NULL;
  return temp;
}
inline void PlanStatus::set_allocated_expect_acc(::sim_msg::AccelerationExpectation* expect_acc) {
  delete expect_acc_;
  expect_acc_ = expect_acc;
  if (expect_acc) {
    
  } else {
    
  }
  // @@protoc_insertion_point(field_set_allocated:sim_msg.PlanStatus.expect_acc)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS
// -------------------------------------------------------------------

// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)


}  // namespace sim_msg

#ifndef SWIG
namespace google {
namespace protobuf {

template <> struct is_proto_enum< ::sim_msg::IndicatorState> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::sim_msg::IndicatorState>() {
  return ::sim_msg::IndicatorState_descriptor();
}
template <> struct is_proto_enum< ::sim_msg::PlanMode> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::sim_msg::PlanMode>() {
  return ::sim_msg::PlanMode_descriptor();
}
template <> struct is_proto_enum< ::sim_msg::PlanClassification> : ::google::protobuf::internal::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::sim_msg::PlanClassification>() {
  return ::sim_msg::PlanClassification_descriptor();
}

}  // namespace protobuf
}  // namespace google
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_planStatus_2eproto__INCLUDED
