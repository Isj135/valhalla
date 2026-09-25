#include "mjolnir/conditional_syntax.h"

#include <cctype>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace valhalla {
namespace mjolnir {
namespace {

struct ClauseView {
  std::string_view text;
  size_t offset;
};

bool is_space(const char c) {
  return std::isspace(static_cast<unsigned char>(c)) != 0;
}

std::string_view trim(std::string_view text) {
  while (!text.empty() && is_space(text.front())) {
    text.remove_prefix(1);
  }
  while (!text.empty() && is_space(text.back())) {
    text.remove_suffix(1);
  }
  return text;
}

ConditionalParseResult error(const ConditionalSyntaxStatus status,
                             const size_t clause,
                             const size_t offset) {
  return {{}, status, clause, offset};
}

} // namespace

ConditionalParseResult parse_conditional_clauses(const std::string_view conditional) {
  std::vector<ClauseView> clause_views;
  size_t clause_start = 0;
  size_t depth = 0;

  for (size_t i = 0; i < conditional.size(); ++i) {
    switch (conditional[i]) {
      case '(':
        ++depth;
        break;
      case ')':
        if (depth == 0) {
          return error(ConditionalSyntaxStatus::kUnbalancedParentheses, clause_views.size(), i);
        }
        --depth;
        break;
      case ';':
        if (depth == 0) {
          const auto clause = conditional.substr(clause_start, i - clause_start);
          if (trim(clause).empty()) {
            return error(ConditionalSyntaxStatus::kEmptyClause, clause_views.size(), clause_start);
          }
          clause_views.push_back({clause, clause_start});
          clause_start = i + 1;
        }
        break;
      default:
        break;
    }
  }

  if (depth != 0) {
    return error(ConditionalSyntaxStatus::kUnbalancedParentheses, clause_views.size(),
                 conditional.size());
  }

  const auto final_clause = conditional.substr(clause_start);
  if (trim(final_clause).empty()) {
    return error(ConditionalSyntaxStatus::kEmptyClause, clause_views.size(), clause_start);
  }
  clause_views.push_back({final_clause, clause_start});

  std::vector<ConditionalClause> clauses;
  clauses.reserve(clause_views.size());
  for (size_t index = 0; index < clause_views.size(); ++index) {
    const auto& clause = clause_views[index];
    size_t separator = std::numeric_limits<size_t>::max();
    depth = 0;

    for (size_t i = 0; i < clause.text.size(); ++i) {
      switch (clause.text[i]) {
        case '(':
          ++depth;
          break;
        case ')':
          --depth;
          break;
        case '@':
          if (depth == 0) {
            if (separator != std::numeric_limits<size_t>::max()) {
              return error(ConditionalSyntaxStatus::kMultipleSeparators, index, clause.offset + i);
            }
            separator = i;
          }
          break;
        default:
          break;
      }
    }

    if (separator == std::numeric_limits<size_t>::max()) {
      return error(ConditionalSyntaxStatus::kMissingSeparator, index, clause.offset);
    }

    const auto value = trim(clause.text.substr(0, separator));
    const auto condition = trim(clause.text.substr(separator + 1));
    if (value.empty()) {
      return error(ConditionalSyntaxStatus::kEmptyValue, index, clause.offset + separator);
    }
    if (condition.empty()) {
      return error(ConditionalSyntaxStatus::kEmptyCondition, index,
                   clause.offset + separator + 1);
    }

    clauses.push_back(
        {index, std::string(value), std::string(condition), ConditionalSyntaxStatus::kValid});
  }

  return {std::move(clauses), ConditionalSyntaxStatus::kValid,
          std::numeric_limits<size_t>::max(), std::numeric_limits<size_t>::max()};
}

} // namespace mjolnir
} // namespace valhalla
