#include "mjolnir/conditional_syntax.h"

#include <gtest/gtest.h>

#include <string_view>

using namespace valhalla::mjolnir;

namespace {

void expect_error(const std::string_view conditional,
                  const ConditionalSyntaxStatus expected_status,
                  const size_t expected_clause) {
  const auto result = parse_conditional_clauses(conditional);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, expected_status);
  EXPECT_EQ(result.error_clause, expected_clause);
  EXPECT_TRUE(result.clauses.empty());
}

} // namespace

TEST(ConditionalSyntax, SplitsTopLevelClausesInOrder) {
  const auto result =
      parse_conditional_clauses("no @ (Mo-Fr 07:30-10:30); yes @ permit");

  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.clauses.size(), 2);
  EXPECT_EQ(result.clauses[0].index, 0);
  EXPECT_EQ(result.clauses[0].value, "no");
  EXPECT_EQ(result.clauses[0].condition, "(Mo-Fr 07:30-10:30)");
  EXPECT_EQ(result.clauses[0].status, ConditionalSyntaxStatus::kValid);
  EXPECT_EQ(result.clauses[1].index, 1);
  EXPECT_EQ(result.clauses[1].value, "yes");
  EXPECT_EQ(result.clauses[1].condition, "permit");
  EXPECT_EQ(result.clauses[1].status, ConditionalSyntaxStatus::kValid);
}

TEST(ConditionalSyntax, PreservesSemicolonsInsideParentheses) {
  const auto result =
      parse_conditional_clauses("no @ (Mo-Fr 06:00-17:00; Sa 06:00-23:00); yes @ permit");

  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.clauses.size(), 2);
  EXPECT_EQ(result.clauses[0].value, "no");
  EXPECT_EQ(result.clauses[0].condition, "(Mo-Fr 06:00-17:00; Sa 06:00-23:00)");
  EXPECT_EQ(result.clauses[1].value, "yes");
  EXPECT_EQ(result.clauses[1].condition, "permit");
}

TEST(ConditionalSyntax, AcceptsClausesWithoutWhitespace) {
  const auto result =
      parse_conditional_clauses("yes@(20:00-07:00);delivery@(Mo-Sa 10:30-12:30)");

  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.clauses.size(), 2);
  EXPECT_EQ(result.clauses[0].value, "yes");
  EXPECT_EQ(result.clauses[0].condition, "(20:00-07:00)");
  EXPECT_EQ(result.clauses[1].value, "delivery");
  EXPECT_EQ(result.clauses[1].condition, "(Mo-Sa 10:30-12:30)");
}

TEST(ConditionalSyntax, TrimsOnlyExternalWhitespace) {
  const auto result = parse_conditional_clauses(" \tno  @  (Mo-Fr 07:30-10:30) \r\n");

  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.clauses.size(), 1);
  EXPECT_EQ(result.clauses[0].value, "no");
  EXPECT_EQ(result.clauses[0].condition, "(Mo-Fr 07:30-10:30)");
}

TEST(ConditionalSyntax, RejectsMalformedInputAtomically) {
  expect_error("", ConditionalSyntaxStatus::kEmptyClause, 0);
  expect_error("; no @ (Mo-Fr)", ConditionalSyntaxStatus::kEmptyClause, 0);
  expect_error("no (Mo-Fr)", ConditionalSyntaxStatus::kMissingSeparator, 0);
  expect_error("no @", ConditionalSyntaxStatus::kEmptyCondition, 0);
  expect_error("@ (Mo-Fr)", ConditionalSyntaxStatus::kEmptyValue, 0);
  expect_error("no @ (Mo-Fr; yes @ permit", ConditionalSyntaxStatus::kUnbalancedParentheses, 0);
  expect_error("no @ (Mo-Fr)); yes @ permit", ConditionalSyntaxStatus::kUnbalancedParentheses, 0);
  expect_error("no @ (Mo-Fr);", ConditionalSyntaxStatus::kEmptyClause, 1);
  expect_error("no @ (Mo-Fr) @ permit", ConditionalSyntaxStatus::kMultipleSeparators, 0);
  expect_error("no @ (Mo-Fr); ; yes @ permit", ConditionalSyntaxStatus::kEmptyClause, 1);
  expect_error("no @ (Mo-Fr); yes permit", ConditionalSyntaxStatus::kMissingSeparator, 1);
}
