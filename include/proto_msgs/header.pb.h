// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: header.proto

#ifndef PROTOBUF_header_2eproto__INCLUDED
#define PROTOBUF_header_2eproto__INCLUDED

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
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
namespace sim_msg {
class Header;
class HeaderDefaultTypeInternal;
extern HeaderDefaultTypeInternal _Header_default_instance_;
}  // namespace sim_msg

namespace sim_msg {

namespace protobuf_header_2eproto {
// Internal implementation detail -- do not call these.
struct TableStruct {
  static const ::google::protobuf::uint32 offsets[];
  static void InitDefaultsImpl();
  static void Shutdown();
};
void AddDescriptors();
void InitDefaults();
}  // namespace protobuf_header_2eproto

// ===================================================================

class Header : public ::google::protobuf::Message /* @@protoc_insertion_point(class_definition:sim_msg.Header) */ {
 public:
  Header();
  virtual ~Header();

  Header(const Header& from);

  inline Header& operator=(const Header& from) {
    CopyFrom(from);
    return *this;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const Header& default_instance();

  static inline const Header* internal_default_instance() {
    return reinterpret_cast<const Header*>(
               &_Header_default_instance_);
  }

  void Swap(Header* other);

  // implements Message ----------------------------------------------

  inline Header* New() const PROTOBUF_FINAL { return New(NULL); }

  Header* New(::google::protobuf::Arena* arena) const PROTOBUF_FINAL;
  void CopyFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void MergeFrom(const ::google::protobuf::Message& from) PROTOBUF_FINAL;
  void CopyFrom(const Header& from);
  void MergeFrom(const Header& from);
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
  void InternalSwap(Header* other);
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

  // double time_stamp = 1;
  void clear_time_stamp();
  static const int kTimeStampFieldNumber = 1;
  double time_stamp() const;
  void set_time_stamp(double value);

  // uint64 frame_id = 2;
  void clear_frame_id();
  static const int kFrameIdFieldNumber = 2;
  ::google::protobuf::uint64 frame_id() const;
  void set_frame_id(::google::protobuf::uint64 value);

  // double sequence = 3;
  void clear_sequence();
  static const int kSequenceFieldNumber = 3;
  double sequence() const;
  void set_sequence(double value);

  // @@protoc_insertion_point(class_scope:sim_msg.Header)
 private:

  ::google::protobuf::internal::InternalMetadataWithArena _internal_metadata_;
  double time_stamp_;
  ::google::protobuf::uint64 frame_id_;
  double sequence_;
  mutable int _cached_size_;
  friend struct  protobuf_header_2eproto::TableStruct;
};
// ===================================================================


// ===================================================================

#if !PROTOBUF_INLINE_NOT_IN_HEADERS
// Header

// double time_stamp = 1;
inline void Header::clear_time_stamp() {
  time_stamp_ = 0;
}
inline double Header::time_stamp() const {
  // @@protoc_insertion_point(field_get:sim_msg.Header.time_stamp)
  return time_stamp_;
}
inline void Header::set_time_stamp(double value) {
  
  time_stamp_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.Header.time_stamp)
}

// uint64 frame_id = 2;
inline void Header::clear_frame_id() {
  frame_id_ = GOOGLE_ULONGLONG(0);
}
inline ::google::protobuf::uint64 Header::frame_id() const {
  // @@protoc_insertion_point(field_get:sim_msg.Header.frame_id)
  return frame_id_;
}
inline void Header::set_frame_id(::google::protobuf::uint64 value) {
  
  frame_id_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.Header.frame_id)
}

// double sequence = 3;
inline void Header::clear_sequence() {
  sequence_ = 0;
}
inline double Header::sequence() const {
  // @@protoc_insertion_point(field_get:sim_msg.Header.sequence)
  return sequence_;
}
inline void Header::set_sequence(double value) {
  
  sequence_ = value;
  // @@protoc_insertion_point(field_set:sim_msg.Header.sequence)
}

#endif  // !PROTOBUF_INLINE_NOT_IN_HEADERS

// @@protoc_insertion_point(namespace_scope)


}  // namespace sim_msg

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_header_2eproto__INCLUDED