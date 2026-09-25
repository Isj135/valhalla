#include "mjolnir/conditional_projection.h"

#include "baldr/timedomain.h"
#include "mjolnir/conditional_syntax.h"
#include "mjolnir/timeparsing.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <regex>
#include <string>
#include <string_view>
#include <vector>

namespace valhalla {
namespace mjolnir {
namespace {

constexpr size_t kMonths = 12;
constexpr size_t kWeekdays = 7;
constexpr size_t kMinutes = 24 * 60;
constexpr size_t kCalendarSize = kMonths * kWeekdays * kMinutes;

using Calendar = std::vector<bool>;

size_t cell(const size_t month, const size_t weekday, const size_t minute) {
  return ((month * kWeekdays) + weekday) * kMinutes + minute;
}

std::string lower(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return text;
}

int specificity(const ConditionalScope scope) {
  switch (scope) {
    case ConditionalScope::kAccess:
      return 0;
    case ConditionalScope::kMotorVehicle:
      return 1;
    case ConditionalScope::kMotorcar:
      return 2;
    case ConditionalScope::kUnsupported:
      return -1;
  }
  return -1;
}

OrdinaryAutoProjection failure(const ProjectionStatus status) {
  return {status, ConditionalScope::kUnsupported, CanonicalPolarity::kTimedDenied, {}, {}};
}

bool is_qualified_temporal_syntax(std::string condition) {
  // T2 intentionally recognizes only the alphabet demonstrated by the eight Turin values.
  static const std::regex allowed_chars(R"(^[()0-9A-Za-z,:;\-\s]+$)");
  static const std::regex time_token(R"(\b[0-9]{1,2}:[0-9]{2}\b)");
  static const std::regex word_token(R"([A-Za-z]+)");
  if (!std::regex_match(condition, allowed_chars)) {
    return false;
  }
  condition = std::regex_replace(condition, time_token, " ");
  const std::array<std::string, 5> qualified_words = {"mo", "fr", "sa", "jul", "aug"};
  for (std::sregex_iterator it(condition.begin(), condition.end(), word_token), end; it != end;
       ++it) {
    const auto word = lower(it->str());
    if (std::find(qualified_words.begin(), qualified_words.end(), word) ==
        qualified_words.end()) {
      return false;
    }
  }
  return true;
}

bool month_active(const baldr::TimeDomain& td, const size_t month) {
  const auto begin = td.begin_month();
  const auto end = td.end_month();
  if (begin == 0 && end == 0) {
    return true;
  }
  if (begin == 0 || end == 0) {
    return false;
  }
  const auto actual = static_cast<uint8_t>(month + 1);
  return begin <= end ? actual >= begin && actual <= end : actual >= begin || actual <= end;
}

bool expand_time_domain(const uint64_t encoded, Calendar& active) {
  const baldr::TimeDomain td(encoded);
  if (td.type() != baldr::kYMD || td.begin_day_dow() != 0 || td.end_day_dow() != 0 ||
      td.begin_week() != 0 || td.end_week() != 0) {
    return false;
  }

  const auto begin = static_cast<size_t>(td.begin_hrs()) * 60 + td.begin_mins();
  const auto end = static_cast<size_t>(td.end_hrs()) * 60 + td.end_mins();
  const bool full_day = begin == 0 && end == 0;
  const bool overnight = !full_day && begin >= end;
  if (overnight && (td.begin_month() != 0 || td.end_month() != 0)) {
    return false;
  }

  const auto dow_mask = td.dow() == 0 ? static_cast<uint8_t>(0x7f) : td.dow();
  for (size_t month = 0; month < kMonths; ++month) {
    if (!month_active(td, month)) {
      continue;
    }
    for (size_t weekday = 0; weekday < kWeekdays; ++weekday) {
      if ((dow_mask & (1u << weekday)) == 0) {
        continue;
      }
      if (full_day) {
        for (size_t minute = 0; minute < kMinutes; ++minute) {
          active[cell(month, weekday, minute)] = true;
        }
      } else if (!overnight) {
        for (size_t minute = begin; minute < end; ++minute) {
          active[cell(month, weekday, minute)] = true;
        }
      } else {
        for (size_t minute = begin; minute < kMinutes; ++minute) {
          active[cell(month, weekday, minute)] = true;
        }
        const auto next_weekday = (weekday + 1) % kWeekdays;
        for (size_t minute = 0; minute < end; ++minute) {
          active[cell(month, next_weekday, minute)] = true;
        }
      }
    }
  }
  return true;
}

ProjectionStatus project_clause(const ConditionalClause& source,
                                ProjectedClause& projected,
                                Calendar& condition) {
  const auto value = lower(source.value);
  if (value == "yes") {
    projected.access = ProjectedAccess::kAllow;
  } else if (value == "no" || value == "delivery") {
    projected.access = ProjectedAccess::kDeny;
  } else {
    return ProjectionStatus::kUnsupportedValue;
  }

  const auto expression = lower(source.condition);
  if (expression == "permit" ||
      expression == "(22:00-06:00 and disabled; emergency)") {
    projected.condition = ProjectedCondition::kNever;
    return ProjectionStatus::kProjected;
  }
  if (!is_qualified_temporal_syntax(source.condition)) {
    return ProjectionStatus::kUnsupportedCondition;
  }

  projected.condition = ProjectedCondition::kTemporal;
  projected.time_domains = get_time_range(source.condition);
  if (projected.time_domains.empty()) {
    return ProjectionStatus::kUnsupportedCondition;
  }
  for (const auto encoded : projected.time_domains) {
    if (!expand_time_domain(encoded, condition)) {
      return ProjectionStatus::kUnrepresentableCalendar;
    }
  }
  return ProjectionStatus::kProjected;
}

std::vector<CanonicalTimeDomain> canonicalize(const Calendar& changed) {
  std::vector<CanonicalTimeDomain> result;
  for (size_t month = 0; month < kMonths; ++month) {
    for (size_t weekday = 0; weekday < kWeekdays; ++weekday) {
      size_t minute = 0;
      while (minute < kMinutes) {
        while (minute < kMinutes && !changed[cell(month, weekday, minute)]) {
          ++minute;
        }
        const auto begin = minute;
        while (minute < kMinutes && changed[cell(month, weekday, minute)]) {
          ++minute;
        }
        if (begin != minute) {
          result.push_back({static_cast<uint8_t>(month + 1), static_cast<uint8_t>(weekday),
                            static_cast<uint16_t>(begin), static_cast<uint16_t>(minute)});
        }
      }
    }
  }
  return result;
}

} // namespace

OrdinaryAutoProjection
project_ordinary_auto(const OrdinaryAutoBase base,
                      const std::vector<ConditionalProgram>& programs) {
  if (base == OrdinaryAutoBase::kUnsupported) {
    return failure(ProjectionStatus::kUnsupportedBase);
  }
  if (programs.empty()) {
    return failure(ProjectionStatus::kUnsupportedScope);
  }

  const ConditionalProgram* selected = nullptr;
  int selected_specificity = -1;
  std::array<bool, 3> seen = {false, false, false};
  for (const auto& program : programs) {
    const auto rank = specificity(program.scope);
    if (rank < 0) {
      return failure(ProjectionStatus::kUnsupportedScope);
    }
    if (seen[rank]) {
      return failure(ProjectionStatus::kDuplicateScope);
    }
    seen[rank] = true;
    if (rank > selected_specificity) {
      selected = &program;
      selected_specificity = rank;
    }
  }

  const auto parsed = parse_conditional_clauses(selected->conditional);
  if (!parsed.ok()) {
    return failure(ProjectionStatus::kSyntaxError);
  }

  const bool base_allowed = base == OrdinaryAutoBase::kAllow;
  Calendar effective(kCalendarSize, base_allowed);
  std::vector<ProjectedClause> clauses;
  clauses.reserve(parsed.clauses.size());

  for (const auto& source : parsed.clauses) {
    ProjectedClause projected{source.index, ProjectedAccess::kDeny,
                              ProjectedCondition::kAlways, {}};
    Calendar condition(kCalendarSize, false);
    const auto status = project_clause(source, projected, condition);
    if (status != ProjectionStatus::kProjected) {
      return failure(status);
    }
    if (projected.condition == ProjectedCondition::kAlways) {
      std::fill(condition.begin(), condition.end(), true);
    }
    if (projected.condition != ProjectedCondition::kNever) {
      const bool allowed = projected.access == ProjectedAccess::kAllow;
      for (size_t i = 0; i < kCalendarSize; ++i) {
        if (condition[i]) {
          effective[i] = allowed; // OSM conditional semantics: last matching value wins.
        }
      }
    }
    clauses.push_back(std::move(projected));
  }

  Calendar changed(kCalendarSize, false);
  for (size_t i = 0; i < kCalendarSize; ++i) {
    changed[i] = effective[i] != base_allowed;
  }
  return {ProjectionStatus::kProjected, selected->scope,
          base_allowed ? CanonicalPolarity::kTimedDenied : CanonicalPolarity::kTimedAllowed,
          std::move(clauses), canonicalize(changed)};
}

bool canonical_domain_active(const CanonicalTimeDomain& domain,
                             const uint8_t month,
                             const uint8_t weekday,
                             const uint16_t minute) {
  return domain.month == month && domain.weekday == weekday &&
         domain.begin_minute <= minute && minute < domain.end_minute;
}

} // namespace mjolnir
} // namespace valhalla
