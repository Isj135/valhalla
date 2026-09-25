#include "mjolnir/conditional_projection.h"
#include "mjolnir/conditional_syntax.h"

#include <gtest/gtest.h>
#include <osmium/io/pbf_input.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/osm/way.hpp>

#include <array>
#include <cstdlib>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

using namespace valhalla::mjolnir;

namespace {

OrdinaryAutoProjection project(const std::string& text,
                               const OrdinaryAutoBase base = OrdinaryAutoBase::kAllow,
                               const ConditionalScope scope = ConditionalScope::kMotorVehicle) {
  return project_ordinary_auto(base, {{scope, text}});
}

bool active(const OrdinaryAutoProjection& projection,
            const uint8_t month,
            const uint8_t weekday,
            const uint16_t hour,
            const uint16_t minute = 0) {
  const auto minute_of_day = static_cast<uint16_t>(hour * 60 + minute);
  for (const auto& domain : projection.domains) {
    if (canonical_domain_active(domain, month, weekday, minute_of_day)) {
      return true;
    }
  }
  return false;
}

void expect_exact_for_both_bases(const std::string& text) {
  const auto allow = project(text, OrdinaryAutoBase::kAllow);
  const auto deny = project(text, OrdinaryAutoBase::kDeny);
  EXPECT_TRUE(allow.ok()) << text;
  EXPECT_TRUE(deny.ok()) << text;
  EXPECT_EQ(allow.polarity, CanonicalPolarity::kTimedDenied);
  EXPECT_EQ(deny.polarity, CanonicalPolarity::kTimedAllowed);
}

int scope_rank(const ConditionalScope scope) {
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

} // namespace

TEST(ConditionalProjection, PermitIsFalseForOrdinaryAuto) {
  const auto result = project("no @ (Mo-Fr 07:30-10:30); yes @ permit");
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.clauses.size(), 2);
  EXPECT_EQ(result.clauses[0].condition, ProjectedCondition::kTemporal);
  EXPECT_EQ(result.clauses[1].condition, ProjectedCondition::kNever);
  EXPECT_TRUE(active(result, 1, 1, 9));
  EXPECT_FALSE(active(result, 1, 1, 11));
  EXPECT_FALSE(active(result, 1, 0, 9));
}

TEST(ConditionalProjection, LastMatchingSummerClauseWins) {
  const auto result = project("no @ (Mo-Fr 08:15-8:45, 16:15-16:45); yes @ (Jul-Aug)");
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(active(result, 1, 1, 8, 30));
  EXPECT_FALSE(active(result, 7, 1, 8, 30));
  EXPECT_FALSE(active(result, 1, 1, 12));
}

TEST(ConditionalProjection, MostSpecificProgramIsTheOnlyProgramEvaluated) {
  const auto result = project_ordinary_auto(
      OrdinaryAutoBase::kAllow,
      {{ConditionalScope::kAccess, "no @ (07:30-20:00)"},
       {ConditionalScope::kMotorVehicle, "no @ (Mo-Fr 07:30-10:30); yes @ permit"},
       {ConditionalScope::kMotorcar, "delivery @ (Mo-Sa 10:30-12:30)"}});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.selected_scope, ConditionalScope::kMotorcar);
  EXPECT_TRUE(active(result, 1, 1, 11));
  EXPECT_FALSE(active(result, 1, 1, 9));
}

TEST(ConditionalProjection, DominantMalformedProgramFailsWithoutFallback) {
  const std::vector<ConditionalProgram> via_della_rocca = {
      {ConditionalScope::kMotorVehicle,
       "no @ (Mo-Fr 07:30-10:30); yes @ permit"},
      {ConditionalScope::kMotorcar, "no @ (Mo-Fr 07:30-10:30); permit"},
  };
  for (const auto base : {OrdinaryAutoBase::kAllow, OrdinaryAutoBase::kDeny}) {
    const auto result = project_ordinary_auto(base, via_della_rocca);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.status, ProjectionStatus::kSyntaxError);
    EXPECT_TRUE(result.clauses.empty());
    EXPECT_TRUE(result.domains.empty());
  }
}

TEST(ConditionalProjection, LessSpecificMalformedProgramCannotOverrideValidDominantProgram) {
  const auto result = project_ordinary_auto(
      OrdinaryAutoBase::kAllow,
      {{ConditionalScope::kAccess, "no @ (Mo-Fr 07:30-10:30); permit"},
       {ConditionalScope::kMotorVehicle,
        "no @ (Mo-Fr 07:30-10:30); yes @ permit"}});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.selected_scope, ConditionalScope::kMotorVehicle);
  EXPECT_TRUE(active(result, 1, 1, 9));
  EXPECT_FALSE(active(result, 1, 1, 11));
}

TEST(ConditionalProjection, StandaloneMalformedProgramFailsClosed) {
  const auto result = project("no @ (7:00-20:00); permit");
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, ProjectionStatus::kSyntaxError);
  EXPECT_TRUE(result.clauses.empty());
  EXPECT_TRUE(result.domains.empty());
}

TEST(ConditionalProjection, CanonicalDomainsHaveOnePolarityAndDoNotOverlap) {
  const auto result = project(
      "no @ (07:30-20:00); delivery @ (Mo-Sa 10:30-12:30); yes @ permit");
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.polarity, CanonicalPolarity::kTimedDenied);
  for (size_t i = 0; i < result.domains.size(); ++i) {
    EXPECT_LT(result.domains[i].begin_minute, result.domains[i].end_minute);
    for (size_t j = i + 1; j < result.domains.size(); ++j) {
      if (result.domains[i].month == result.domains[j].month &&
          result.domains[i].weekday == result.domains[j].weekday) {
        EXPECT_TRUE(result.domains[i].end_minute <= result.domains[j].begin_minute ||
                    result.domains[j].end_minute <= result.domains[i].begin_minute);
      }
    }
  }
}

TEST(ConditionalProjection, OvernightShiftsAfterMidnightToTheNextWeekday) {
  const auto result = project("yes @ (Mo 20:00-07:00)", OrdinaryAutoBase::kDeny,
                              ConditionalScope::kAccess);
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(active(result, 1, 1, 21));
  EXPECT_TRUE(active(result, 1, 2, 6));
  EXPECT_FALSE(active(result, 1, 1, 6));
  EXPECT_FALSE(active(result, 1, 0, 21));
}

TEST(ConditionalProjection, DeliveryDoesNotAuthorizeOrdinaryAuto) {
  const auto result = project("delivery @ (10:30-12:30); yes @ (20:00-07:00)");
  ASSERT_TRUE(result.ok());
  EXPECT_TRUE(active(result, 1, 1, 11));
  EXPECT_FALSE(active(result, 1, 1, 21));
}

TEST(ConditionalProjection, AllEightQualifiedTurinValuesProjectForBothBases) {
  const std::array<std::pair<ConditionalScope, const char*>, 8> values = {{
      {ConditionalScope::kMotorVehicle,
       "no @ (Mo-Fr 07:30-10:30); yes @ permit"},
      {ConditionalScope::kMotorVehicle,
       "delivery @ (10:30-12:30); yes @ (20:00-07:00)"},
      {ConditionalScope::kMotorVehicle,
       "no @ (Mo-Fr 06:00-17:00; Sa 06:00-23:00); yes @ permit"},
      {ConditionalScope::kMotorVehicle, "no @ (07:30-20:00); yes @ permit"},
      {ConditionalScope::kMotorVehicle,
       "no @ (07:30-20:00); delivery @ (Mo-Sa 10:30-12:30); yes @ permit"},
      {ConditionalScope::kAccess,
       "yes@ (20:00-07:00); delivery @ (Mo-Sa 10:30-12:30)"},
      {ConditionalScope::kAccess,
       "no @ (Mo-Fr 08:15-8:45, 16:15-16:45); yes @ (Jul-Aug)"},
      {ConditionalScope::kAccess,
       "no @ (22:00-06:00); yes @ (22:00-06:00 AND disabled; emergency)"},
  }};

  for (const auto& value : values) {
    const auto allow = project(value.second, OrdinaryAutoBase::kAllow, value.first);
    const auto deny = project(value.second, OrdinaryAutoBase::kDeny, value.first);
    EXPECT_TRUE(allow.ok()) << value.second;
    EXPECT_TRUE(deny.ok()) << value.second;
    EXPECT_EQ(allow.polarity, CanonicalPolarity::kTimedDenied);
    EXPECT_EQ(deny.polarity, CanonicalPolarity::kTimedAllowed);
  }
}

TEST(ConditionalProjection, UnsupportedInputsFailWithoutAProjection) {
  const std::array<std::pair<OrdinaryAutoProjection, ProjectionStatus>, 8> failures = {{
      {project("private @ (Mo-Fr 07:30-10:30)"), ProjectionStatus::kUnsupportedValue},
      {project("yes @ resident"), ProjectionStatus::kUnsupportedCondition},
      {project("yes @ (Su 10:00-11:00)"), ProjectionStatus::kUnsupportedCondition},
      {project("no (Mo-Fr)"), ProjectionStatus::kSyntaxError},
      {project_ordinary_auto(OrdinaryAutoBase::kUnsupported,
                             {{ConditionalScope::kAccess, "no @ (07:30-20:00)"}}),
       ProjectionStatus::kUnsupportedBase},
      {project("no @ (Jul-Aug 20:00-07:00)"),
       ProjectionStatus::kUnrepresentableCalendar},
      {project("no @ (07:30-20:00)", OrdinaryAutoBase::kAllow,
               ConditionalScope::kUnsupported),
       ProjectionStatus::kUnsupportedScope},
      {project_ordinary_auto(OrdinaryAutoBase::kAllow,
                             {{ConditionalScope::kAccess, "no @ (07:30-20:00)"},
                              {ConditionalScope::kAccess, "yes @ (20:00-21:00)"}}),
       ProjectionStatus::kDuplicateScope},
  }};
  for (const auto& failure : failures) {
    EXPECT_FALSE(failure.first.ok());
    EXPECT_EQ(failure.first.status, failure.second);
    EXPECT_TRUE(failure.first.clauses.empty());
    EXPECT_TRUE(failure.first.domains.empty());
  }
}

TEST(ConditionalProjection, TurinFrequencyCensusClosesAt672Ways) {
  const std::array<size_t, 8> value_counts = {618, 22, 14, 9, 3, 8, 2, 1};
  const size_t overlapping_multi_tag_occurrences = 5;
  size_t occurrences = 0;
  for (const auto count : value_counts) {
    occurrences += count;
  }
  EXPECT_EQ(occurrences - overlapping_multi_tag_occurrences, 672);
}

TEST(ConditionalProjection, QualifiedTurinPbfCensus) {
  const auto* pbf = std::getenv("VALHALLA_T2_TURIN_PBF");
  if (pbf == nullptr || *pbf == '\0') {
    GTEST_SKIP() << "Set VALHALLA_T2_TURIN_PBF to run the read-only dataset census";
  }

  const std::unordered_set<std::string> qualified_values = {
      "no @ (Mo-Fr 07:30-10:30); yes @ permit",
      "delivery @ (10:30-12:30); yes @ (20:00-07:00)",
      "no @ (Mo-Fr 06:00-17:00; Sa 06:00-23:00); yes @ permit",
      "no @ (07:30-20:00); yes @ permit",
      "no @ (07:30-20:00); delivery @ (Mo-Sa 10:30-12:30); yes @ permit",
      "yes@ (20:00-07:00); delivery @ (Mo-Sa 10:30-12:30)",
      "no @ (Mo-Fr 08:15-8:45, 16:15-16:45); yes @ (Jul-Aug)",
      "no @ (22:00-06:00); yes @ (22:00-06:00 AND disabled; emergency)",
  };
  const std::map<std::string, ConditionalScope> recognized_keys = {
      {"access:conditional", ConditionalScope::kAccess},
      {"motor_vehicle:conditional", ConditionalScope::kMotorVehicle},
      {"motorcar:conditional", ConditionalScope::kMotorcar},
  };

  size_t occurrences = 0;
  size_t directly_qualified = 0;
  size_t lower_malformed_but_qualified = 0;
  size_t projected = 0;
  size_t fail_closed = 0;
  size_t unknown = 0;
  size_t multi_tag_ways = 0;
  bool standalone_malformed_seen = false;
  std::unordered_set<osmium::object_id_type> way_ids;
  osmium::io::Reader reader(pbf, osmium::osm_entity_bits::way);
  while (const osmium::memory::Buffer buffer = reader.read()) {
    for (const osmium::memory::Item& item : buffer) {
      const auto& way = static_cast<const osmium::Way&>(item);
      std::vector<ConditionalProgram> programs;
      size_t qualified_on_way = 0;
      for (const auto& tag : way.tags()) {
        const auto key = recognized_keys.find(tag.key());
        if (key == recognized_keys.end()) {
          continue;
        }
        programs.push_back({key->second, tag.value()});
        if (qualified_values.count(tag.value()) != 0) {
          ++qualified_on_way;
          ++occurrences;
        }
      }
      if (way.id() == 133198388) {
        const auto standalone = project_ordinary_auto(OrdinaryAutoBase::kAllow, programs);
        EXPECT_EQ(standalone.status, ProjectionStatus::kSyntaxError);
        EXPECT_TRUE(standalone.clauses.empty());
        EXPECT_TRUE(standalone.domains.empty());
        standalone_malformed_seen = true;
      }
      if (qualified_on_way == 0) {
        continue;
      }
      way_ids.insert(way.id());
      if (qualified_on_way > 1) {
        ++multi_tag_ways;
      }
      const auto allow = project_ordinary_auto(OrdinaryAutoBase::kAllow, programs);
      const auto deny = project_ordinary_auto(OrdinaryAutoBase::kDeny, programs);
      const auto selected = std::max_element(
          programs.begin(), programs.end(), [](const auto& left, const auto& right) {
            return scope_rank(left.scope) < scope_rank(right.scope);
          });
      const bool selected_malformed = !parse_conditional_clauses(selected->conditional).ok();
      bool lower_malformed = false;
      for (const auto& program : programs) {
        if (scope_rank(program.scope) < scope_rank(selected->scope) &&
            !parse_conditional_clauses(program.conditional).ok()) {
          lower_malformed = true;
        }
      }

      if (selected_malformed) {
        if (way.id() != 1479446338 || allow.status != ProjectionStatus::kSyntaxError ||
            deny.status != ProjectionStatus::kSyntaxError || !allow.clauses.empty() ||
            !deny.clauses.empty() || !allow.domains.empty() || !deny.domains.empty()) {
          ++unknown;
        } else {
          ++fail_closed;
        }
        continue;
      }
      if (!allow.ok() || !deny.ok()) {
        ++unknown;
        continue;
      }
      ++projected;
      if (lower_malformed) {
        ++lower_malformed_but_qualified;
      } else {
        ++directly_qualified;
      }
    }
  }
  reader.close();

  EXPECT_EQ(occurrences, 677);
  EXPECT_EQ(way_ids.size(), 672);
  EXPECT_EQ(directly_qualified, 666);
  EXPECT_EQ(lower_malformed_but_qualified, 5);
  EXPECT_EQ(fail_closed, 1);
  EXPECT_EQ(projected, 671);
  EXPECT_EQ(unknown, 0);
  EXPECT_EQ(multi_tag_ways, 5);
  EXPECT_TRUE(standalone_malformed_seen);
  EXPECT_EQ(directly_qualified + lower_malformed_but_qualified + fail_closed + unknown, 672);
}
