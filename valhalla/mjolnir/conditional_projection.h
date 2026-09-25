#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace valhalla {
namespace mjolnir {

enum class OrdinaryAutoBase : uint8_t { kAllow, kDeny, kUnsupported };
enum class ConditionalScope : uint8_t { kAccess, kMotorVehicle, kMotorcar, kUnsupported };
enum class ProjectedAccess : uint8_t { kAllow, kDeny };
enum class ProjectedCondition : uint8_t { kAlways, kNever, kTemporal };
enum class CanonicalPolarity : uint8_t { kTimedDenied, kTimedAllowed };

enum class ProjectionStatus : uint8_t {
  kProjected,
  kSyntaxError,
  kUnsupportedValue,
  kUnsupportedCondition,
  kUnrepresentableCalendar,
  kUnsupportedBase,
  kUnsupportedScope,
  kDuplicateScope,
};

struct ConditionalProgram {
  ConditionalScope scope;
  std::string conditional;
};

struct ProjectedClause {
  size_t index;
  ProjectedAccess access;
  ProjectedCondition condition;
  std::vector<uint64_t> time_domains;
};

// A disjoint civil-time cell. month is 1..12, weekday is 0=Sunday..6=Saturday,
// and the minute interval is half-open [begin_minute, end_minute).
struct CanonicalTimeDomain {
  uint8_t month;
  uint8_t weekday;
  uint16_t begin_minute;
  uint16_t end_minute;
};

struct OrdinaryAutoProjection {
  ProjectionStatus status;
  ConditionalScope selected_scope;
  CanonicalPolarity polarity;
  std::vector<ProjectedClause> clauses;
  std::vector<CanonicalTimeDomain> domains;

  bool ok() const {
    return status == ProjectionStatus::kProjected;
  }
};

OrdinaryAutoProjection
project_ordinary_auto(OrdinaryAutoBase base, const std::vector<ConditionalProgram>& programs);

bool canonical_domain_active(const CanonicalTimeDomain& domain,
                             uint8_t month,
                             uint8_t weekday,
                             uint16_t minute);

} // namespace mjolnir
} // namespace valhalla
