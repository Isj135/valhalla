#include "baldr/graphconstants.h"
#include "baldr/timedomain.h"
#include "midgard/sequence.h"
#include "mjolnir/osmaccessrestriction.h"
#include "mjolnir/osmway.h"
#include "mjolnir/pbfgraphparser.h"

#include <boost/property_tree/ptree.hpp>
#include <gtest/gtest.h>
#include <osmium/io/pbf_input.hpp>

#include <array>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

using namespace valhalla::baldr;
using namespace valhalla::midgard;
using namespace valhalla::mjolnir;

namespace {

constexpr uint16_t kHistoricalMotorVehicleModes =
    kAutoAccess | kTruckAccess | kEmergencyAccess | kTaxiAccess | kBusAccess | kHOVAccess |
    kMopedAccess | kMotorcycleAccess;
constexpr std::array<uint64_t, 4> kVanchigliaWays = {
    133198392, 1368813773, 633589514, 1316009421};

const std::unordered_set<std::string> kQualifiedValues = {
    "no @ (Mo-Fr 07:30-10:30); yes @ permit",
    "delivery @ (10:30-12:30); yes @ (20:00-07:00)",
    "no @ (Mo-Fr 06:00-17:00; Sa 06:00-23:00); yes @ permit",
    "no @ (07:30-20:00); yes @ permit",
    "no @ (07:30-20:00); delivery @ (Mo-Sa 10:30-12:30); yes @ permit",
    "yes@ (20:00-07:00); delivery @ (Mo-Sa 10:30-12:30)",
    "no @ (Mo-Fr 08:15-8:45, 16:15-16:45); yes @ (Jul-Aug)",
    "no @ (22:00-06:00); yes @ (22:00-06:00 AND disabled; emergency)",
};

bool is_ordinary_auto_key(const char* key) {
  return std::string{key} == "access:conditional" ||
         std::string{key} == "motor_vehicle:conditional" ||
         std::string{key} == "motorcar:conditional";
}

OSMWay find_way(sequence<OSMWay>& ways, const uint64_t id) {
  for (size_t i = 0; i < ways.size(); ++i) {
    const auto& way = *ways[i];
    if (way.way_id() == id) {
      return way;
    }
  }
  throw std::runtime_error("way not found: " + std::to_string(id));
}

std::vector<OSMAccessRestriction> auto_restrictions(const OSMData& osmdata, const uint64_t id) {
  std::vector<OSMAccessRestriction> result;
  const auto range = osmdata.access_restrictions.equal_range(id);
  for (auto item = range.first; item != range.second; ++item) {
    if ((item->second.modes() & kAutoAccess) != 0) {
      result.push_back(item->second);
    }
  }
  return result;
}

void expect_weekday_morning(const OSMAccessRestriction& restriction, const uint16_t modes) {
  EXPECT_EQ(restriction.type(), AccessType::kTimedDenied);
  EXPECT_EQ(restriction.modes(), modes);
  const TimeDomain td(restriction.value());
  EXPECT_EQ(td.type(), kYMD);
  EXPECT_EQ(td.dow(), 62);
  EXPECT_EQ(td.begin_hrs(), 7);
  EXPECT_EQ(td.begin_mins(), 30);
  EXPECT_EQ(td.end_hrs(), 10);
  EXPECT_EQ(td.end_mins(), 30);
  EXPECT_EQ(td.begin_month(), 0);
  EXPECT_EQ(td.end_month(), 0);
}

struct TempTree {
  std::filesystem::path path;
  ~TempTree() {
    std::error_code ec;
    std::filesystem::remove_all(path, ec);
  }
};

} // namespace

TEST(OrdinaryAutoIntegration, TurinPbfParserAndIntermediateRestrictions) {
  const auto* pbf = std::getenv("VALHALLA_T3_TURIN_PBF");
  if (pbf == nullptr || *pbf == '\0') {
    GTEST_SKIP() << "Set VALHALLA_T3_TURIN_PBF to run the T3 integration gate";
  }

  std::set<uint64_t> qualified_way_ids;
  std::vector<uint64_t> rossini_way_ids;
  bool standalone_malformed_seen = false;
  osmium::io::Reader reader(pbf, osmium::osm_entity_bits::way);
  while (const osmium::memory::Buffer buffer = reader.read()) {
    for (const osmium::memory::Item& item : buffer) {
      const auto& way = static_cast<const osmium::Way&>(item);
      bool qualified = false;
      bool rossini = false;
      for (const auto& tag : way.tags()) {
        if (!is_ordinary_auto_key(tag.key())) {
          continue;
        }
        qualified = qualified || kQualifiedValues.count(tag.value()) != 0;
        rossini = rossini ||
                  (std::string{tag.key()} == "motor_vehicle:conditional" &&
                   std::string{tag.value()} == "no @ (Mo-Fr 07:30-10:30)");
      }
      if (qualified) {
        qualified_way_ids.insert(way.id());
      }
      if (rossini && std::string{way.tags()["name"]} == "Via Gioachino Rossini") {
        rossini_way_ids.push_back(way.id());
      }
      standalone_malformed_seen = standalone_malformed_seen || way.id() == 133198388;
    }
  }
  reader.close();

  ASSERT_EQ(qualified_way_ids.size(), 672);
  ASSERT_FALSE(rossini_way_ids.empty());
  ASSERT_TRUE(standalone_malformed_seen);

  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  TempTree temp{std::filesystem::temp_directory_path() /
                ("valhalla_t3_ordinary_auto_" + std::to_string(nonce))};
  std::filesystem::create_directories(temp.path);
  const auto ways_file = (temp.path / "ways.bin").string();
  const auto way_nodes_file = (temp.path / "way_nodes.bin").string();
  const auto access_file = (temp.path / "access.bin").string();

  boost::property_tree::ptree config;
  config.put("id_table_size", 1000);
  config.put("tile_dir", (temp.path / "tiles").string());
  auto osmdata =
      PBFGraphParser::ParseWays(config, {pbf}, ways_file, way_nodes_file, access_file);
  sequence<OSMWay> ways(ways_file, false);

  size_t projected_with_auto_restrictions = 0;
  size_t projected_canonical_noops = 0;
  for (const auto id : qualified_way_ids) {
    const auto restrictions = auto_restrictions(osmdata, id);
    if (id == 1479446338) {
      const auto way = find_way(ways, id);
      EXPECT_FALSE(way.auto_forward());
      EXPECT_FALSE(way.auto_backward());
      EXPECT_TRUE(way.taxi_forward());
      EXPECT_TRUE(way.bus_forward());
      EXPECT_FALSE(way.emergency_forward());
      EXPECT_TRUE(restrictions.empty());
    } else {
      if (restrictions.empty()) {
        ++projected_canonical_noops;
      } else {
        ++projected_with_auto_restrictions;
      }
    }
  }
  EXPECT_EQ(projected_with_auto_restrictions, 656);
  EXPECT_EQ(projected_canonical_noops, 15);
  EXPECT_EQ(projected_with_auto_restrictions + projected_canonical_noops, 671);

  for (const auto id : kVanchigliaWays) {
    const auto way = find_way(ways, id);
    EXPECT_TRUE(way.auto_forward());
    const auto restrictions = auto_restrictions(osmdata, id);
    ASSERT_EQ(restrictions.size(), 1) << id;
    expect_weekday_morning(restrictions.front(), kAutoAccess);
    EXPECT_NE(restrictions.front().direction(), AccessRestrictionDirection::kBackward);
  }

  for (const auto id : rossini_way_ids) {
    const auto restrictions = auto_restrictions(osmdata, id);
    ASSERT_FALSE(restrictions.empty()) << id;
    bool historical_record_found = false;
    for (const auto& restriction : restrictions) {
      if (restriction.modes() == kHistoricalMotorVehicleModes) {
        expect_weekday_morning(restriction, kHistoricalMotorVehicleModes);
        historical_record_found = true;
      }
    }
    EXPECT_TRUE(historical_record_found) << id;
  }

  const auto standalone = find_way(ways, 133198388);
  EXPECT_TRUE(standalone.auto_forward());
  EXPECT_FALSE(standalone.auto_backward());
  const auto historical = auto_restrictions(osmdata, 133198388);
  ASSERT_EQ(historical.size(), 1);

  const auto& restriction = historical.front();
  EXPECT_EQ(restriction.type(), AccessType::kTimedDenied);
  EXPECT_EQ(restriction.modes(), kHistoricalMotorVehicleModes);
  EXPECT_EQ(restriction.direction(), AccessRestrictionDirection::kBoth);
  EXPECT_FALSE(restriction.except_destination());
  EXPECT_EQ(restriction.value(), 42949674752ULL);

  const TimeDomain td(restriction.value());
  EXPECT_EQ(td.type(), kYMD);
  EXPECT_EQ(td.dow(), 0);
  EXPECT_EQ(td.begin_hrs(), 7);
  EXPECT_EQ(td.begin_mins(), 0);
  EXPECT_EQ(td.end_hrs(), 20);
  EXPECT_EQ(td.end_mins(), 0);
  EXPECT_EQ(td.begin_month(), 0);
  EXPECT_EQ(td.end_month(), 0);
  EXPECT_EQ(td.begin_day_dow(), 0);
  EXPECT_EQ(td.end_day_dow(), 0);
  EXPECT_EQ(td.begin_week(), 0);
  EXPECT_EQ(td.end_week(), 0);
}
