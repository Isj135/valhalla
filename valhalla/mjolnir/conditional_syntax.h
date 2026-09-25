#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace valhalla {
namespace mjolnir {

enum class ConditionalSyntaxStatus : uint8_t {
  kValid,
  kEmptyClause,
  kUnbalancedParentheses,
  kMissingSeparator,
  kMultipleSeparators,
  kEmptyValue,
  kEmptyCondition,
};

struct ConditionalClause {
  size_t index;
  std::string value;
  std::string condition;
  ConditionalSyntaxStatus status;
};

struct ConditionalParseResult {
  std::vector<ConditionalClause> clauses;
  ConditionalSyntaxStatus status;
  size_t error_clause;
  size_t error_offset;

  bool ok() const {
    return status == ConditionalSyntaxStatus::kValid;
  }
};

ConditionalParseResult parse_conditional_clauses(std::string_view conditional);

} // namespace mjolnir
} // namespace valhalla
