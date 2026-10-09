#include "baldr/rapidjson_utils.h"
#include "loki/worker.h"
#include "gurka.h"
#include "test.h"
#include "thor/unidirectional_astar.h"
#include "thor/worker.h"
#include "odin/worker.h"
#include "midgard/util.h"
#include "midgard/encoded.h"
#include "worker.h"

#include <boost/property_tree/json_parser.hpp>
#include <cmath>
#include <algorithm>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <stdexcept>
#include <tuple>

using namespace valhalla;
using namespace valhalla::thor;
using namespace valhalla::sif;
using namespace valhalla::loki;
using namespace valhalla::baldr;
using namespace valhalla::midgard;

namespace {
// Maximum edge score - base this on costing type.
// Large values can cause very bad performance. Setting this back
// to 2 hours for bike and pedestrian and 12 hours for driving routes.
// TODO - re-evaluate edge scores and balance performance vs. quality.
// Perhaps tie the edge score logic in with the costing type - but
// may want to do this in loki. At this point in thor the costing method
// has not yet been constructed.
const std::unordered_map<std::string, float> kMaxDistances = {
    {"auto", 43200.0f},      {"auto_shorter", 43200.0f},  {"bicycle", 7200.0f},
    {"bus", 43200.0f},       {"motor_scooter", 14400.0f}, {"multimodal", 7200.0f},
    {"pedestrian", 7200.0f}, {"transit", 14400.0f},       {"truck", 43200.0f},
    {"taxi", 43200.0f},
};

const auto cfg = test::make_config(VALHALLA_BUILD_DIR "test/data/utrecht_tiles");

void try_path(GraphReader& reader,
              loki_worker_t& loki_worker,
              const bool depart_at,
              const char* test_request,
              const uint32_t expected_edgecount) {
  Api request;
  ParseApi(test_request, Options::route, request);
  loki_worker.route(request);
  loki_worker.cleanup();
  thor_worker_t::adjust_locations(request);

  // For now this just tests auto costing - could extend to other
  travel_mode_t mode;
  auto mode_costing = sif::CostFactory().CreateModeCosting(request.options(), mode);

  valhalla::Location origin = request.options().locations(0);
  valhalla::Location dest = request.options().locations(1);
  if (depart_at) {
    TimeDepForward alg;
    auto pathedges = alg.GetBestPath(origin, dest, reader, mode_costing, mode).front();
    EXPECT_EQ(pathedges.size(), expected_edgecount) << "Depart at path failed";
  } else {
    TimeDepReverse alg;
    auto pathedges = alg.GetBestPath(origin, dest, reader, mode_costing, mode).front();
    EXPECT_EQ(pathedges.size(), expected_edgecount) << "Arrive by path failed";
  }
}

} // namespace

TEST(TimeDepPaths, test_depart_at_paths) {
  // Test setup
  loki_worker_t loki_worker(cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  // Simple path along oneway edge in the drivable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto","date_time":{"type":1,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, true, test_request1, 1);

  // Simple path along oneway edge opposing the drivable direction -must not
  // return a single edge (edge count is 10)
  const auto test_request2 = R"({"locations":[{"lat":52.078937,"lon":5.115321},
               {"lat":52.079079,"lon":5.115197}],"costing":"auto","date_time":{"type":1,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, true, test_request2, 10);
}

TEST(TimeDepPaths, test_arrive_by_paths) {
  // Test setup
  loki_worker_t loki_worker(cfg);
  GraphReader reader(cfg.get_child("mjolnir"));

  // Simple path along oneway edge in the drivable direction - should return a single edge
  const auto test_request1 = R"({"locations":[{"lat":52.079079,"lon":5.115197},
               {"lat":52.078937,"lon":5.115321}],"costing":"auto","date_time":{"type":2,"value":"2018-06-28T07:00"}})";
  try_path(reader, loki_worker, false, test_request1, 1);
}

class TimeDepForwardTest : public thor::TimeDepForward {
public:
  explicit TimeDepForwardTest(const boost::property_tree::ptree& config = {})
      : TimeDepForward(config) {
  }

  void Clear() {
    TimeDepForward::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(edgelabels_.capacity(), 0);
    } else {
      EXPECT_LE(edgelabels_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(TimeDepPaths, test_forward_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  TimeDepForwardTest time_dep(config);
  time_dep.Clear();
}

TEST(TimeDepPaths, test_forward_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_astar", 10);

  TimeDepForwardTest time_dep(config);
  time_dep.Clear();
}

class TimeDepReverseTest : public thor::TimeDepReverse {
public:
  explicit TimeDepReverseTest(const boost::property_tree::ptree& config = {})
      : TimeDepReverse(config) {
  }

  void Clear() {
    TimeDepReverse::Clear();
    if (clear_reserved_memory_) {
      EXPECT_EQ(edgelabels_.capacity(), 0);
    } else {
      EXPECT_LE(edgelabels_.capacity(), max_reserved_labels_count_);
    }
  }
};

TEST(TimeDepPaths, test_reverse_clear_reserved_memory) {
  boost::property_tree::ptree config;
  config.put("clear_reserved_memory", true);

  TimeDepReverseTest time_dep(config);
  time_dep.Clear();
}

TEST(TimeDepPaths, test_reverse_max_reserved_labels_count) {
  boost::property_tree::ptree config;
  config.put("max_reserved_labels_count_dijkstras", 10);

  TimeDepReverseTest time_dep(config);
  time_dep.Clear();
}

// Frozen expected contract: valhalla-pre-diff-rev2/expected.json, 09 October 2026.
// These tests inspect diagnostic custody, not an alternative route response.
namespace {
using Snapshot = TimeDepForward::ExperimentalChainSnapshot;

auto label_values(const BDEdgeLabel& label) {
  return std::make_tuple(label.predecessor(), label.edgeid().value, label.endnode().value,
                         label.mode(), label.use(), label.origin(), label.destination(),
                         label.shortcut(), label.cost().cost, label.cost().secs,
                         label.sortcost(), label.path_distance(), label.restriction_idx(),
                         label.transition_cost().cost, label.transition_cost().secs,
                         label.opp_edgeid().value, label.not_thru_pruning());
}

auto snapshot_values(const Snapshot& snapshot) {
  std::vector<decltype(label_values(BDEdgeLabel{}))> values;
  for (const auto& step : snapshot.steps)
    values.push_back(label_values(step.label));
  return values;
}

void expect_same_paths(const std::vector<std::vector<PathInfo>>& expected,
                       const std::vector<std::vector<PathInfo>>& actual) {
  ASSERT_EQ(expected.size(), actual.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    ASSERT_EQ(expected[i].size(), actual[i].size());
    for (size_t j = 0; j < expected[i].size(); ++j) {
      const auto& a = expected[i][j];
      const auto& b = actual[i][j];
      EXPECT_EQ(a.edgeid, b.edgeid);
      EXPECT_EQ(a.mode, b.mode);
      EXPECT_EQ(a.elapsed_cost.cost, b.elapsed_cost.cost);
      EXPECT_EQ(a.elapsed_cost.secs, b.elapsed_cost.secs);
      EXPECT_EQ(a.path_distance, b.path_distance);
      EXPECT_EQ(a.restriction_index, b.restriction_index);
      EXPECT_EQ(a.transition_cost.cost, b.transition_cost.cost);
      EXPECT_EQ(a.transition_cost.secs, b.transition_cost.secs);
      EXPECT_EQ(a.trip_id, b.trip_id);
      EXPECT_EQ(a.is_shortcut, b.is_shortcut);
      EXPECT_EQ(a.is_disconnected, b.is_disconnected);
      EXPECT_EQ(a.start_node_is_recovered, b.start_node_is_recovered);
    }
  }
}

// Manual labels test value custody only: no graph, expansion or costing is traversed.
class ExperimentalAlgorithm : public TimeDepForward {
public:
  Location origin;
  Location destination;

  static BDEdgeLabel label(uint32_t index, uint32_t predecessor,
                           Use use = Use::kRoad, bool shortcut = false,
                           sif::TravelMode mode = sif::TravelMode::kDrive) {
    DirectedEdge edge;
    edge.set_endnode(GraphId(100, 0, index + 1));
    edge.set_use(use);
    edge.set_length(100);
    if (shortcut)
      edge.set_shortcut(1);
    const float cost = 15 + 20 * index;
    BDEdgeLabel result(predecessor, GraphId(100, 0, index), &edge,
                       {cost, cost}, cost, 0, mode, kInvalidRestriction,
                       true, false, InternalTurn::kNoTurn);
    result.Update(predecessor, {cost, cost}, cost, {}, 75 + 100 * index,
                  kInvalidRestriction);
    if (index == 0)
      result.set_origin();
    return result;
  }

  void seed(size_t count = 2, Use last_use = Use::kRoad) {
    Clear();
    origin.Clear();
    destination.Clear();
    origin.mutable_ll()->set_lat(0);
    origin.mutable_ll()->set_lng(0);
    auto* edge = origin.mutable_correlation()->add_edges();
    edge->set_graph_id(GraphId(100, 0, 0).value);
    edge->set_percent_along(.25);
    edge->mutable_ll()->set_lat(0);
    edge->mutable_ll()->set_lng(0);
    destination.mutable_ll()->set_lat(0);
    destination.mutable_ll()->set_lng(.003);
    for (uint32_t i = 0; i < count; ++i)
      edgelabels_.push_back(label(i, i == 0 ? kInvalidLabel : i - 1,
                                 i + 1 == count ? last_use : Use::kRoad));
    experimental_candidate_ = static_cast<uint32_t>(count - 1);
    experimental_arrival_ = {.002, 0};
    experimental_active_ = true;
  }
  const std::vector<BDEdgeLabel>& labels() const { return edgelabels_; }
  BDEdgeLabel& mutable_label(size_t index) { return edgelabels_.at(index); }
  uint32_t candidate() const { return experimental_candidate_; }
  void ferry(bool value) { has_ferry_ = value; }
};

TEST(ExperimentalChain, ManualValuesAndPartialOriginMetadata) {
  ExperimentalAlgorithm algorithm;
  EXPECT_FALSE(algorithm.HasExperimentalChainObserver());
  algorithm.seed();
  const auto& readonly = algorithm;
  auto snapshot = readonly.CopyExperimentalChain(algorithm.origin, algorithm.destination);
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->steps.size(), 2);
  EXPECT_EQ(snapshot->steps[0].label_index, 0);
  EXPECT_EQ(snapshot->steps[1].label_index, 1);
  EXPECT_EQ(snapshot->steps[0].label.predecessor(), kInvalidLabel);
  EXPECT_EQ(snapshot->steps[1].label.predecessor(), 0);
  EXPECT_EQ(snapshot->steps[0].label.cost().secs, 15);
  EXPECT_EQ(snapshot->steps[1].label.cost().secs, 35);
  EXPECT_EQ(snapshot->steps[0].label.path_distance(), 75);
  EXPECT_EQ(snapshot->steps[1].label.path_distance(), 175);
  EXPECT_EQ(snapshot->origin_edge.percent_along(), .25);
  EXPECT_EQ(snapshot->origin_edge.graph_id(), GraphId(100, 0, 0).value);
  EXPECT_EQ(snapshot->steps.back().label.endnode(), GraphId(100, 0, 2));
  EXPECT_EQ(snapshot->requested_destination.lng(), .003);
  EXPECT_EQ(snapshot->arrival.lng(), .002);
  EXPECT_EQ(snapshot->pass, 0);
}

TEST(ExperimentalChain, ManuallyUpdatedPredecessorUsesLabelIndices) {
  ExperimentalAlgorithm algorithm;
  algorithm.seed(3);
  auto& last = algorithm.mutable_label(2);
  last.Update(0, {35, 35}, 35, {}, 175, kInvalidRestriction);
  auto snapshot = algorithm.CopyExperimentalChain(algorithm.origin, algorithm.destination);
  ASSERT_TRUE(snapshot);
  ASSERT_EQ(snapshot->steps.size(), 2);
  EXPECT_EQ(snapshot->steps[0].label_index, 0);
  EXPECT_EQ(snapshot->steps[1].label_index, 2);
  EXPECT_EQ(snapshot->steps[1].label.predecessor(), 0);
}

TEST(ExperimentalChain, ConstCopyDoesNotChangeFerryOrLabels) {
  for (const bool ferry : {false, true}) {
    ExperimentalAlgorithm algorithm;
    algorithm.seed(2, Use::kFerry);
    algorithm.ferry(ferry);
    const auto before = algorithm.labels();
    const auto candidate = algorithm.candidate();
    const auto& readonly = algorithm;
    auto snapshot = readonly.CopyExperimentalChain(algorithm.origin, algorithm.destination);
    ASSERT_TRUE(snapshot);
    EXPECT_EQ(algorithm.has_ferry(), ferry);
    EXPECT_EQ(algorithm.candidate(), candidate);
    ASSERT_EQ(algorithm.labels().size(), before.size());
    for (size_t i = 0; i < before.size(); ++i)
      EXPECT_EQ(label_values(algorithm.labels()[i]), label_values(before[i]));
  }
}

TEST(ExperimentalChain, RejectsInvalidChainsWithoutWrites) {
  for (int invalid_case = 0; invalid_case < 9; ++invalid_case) {
    ExperimentalAlgorithm algorithm;
    algorithm.seed();
    switch (invalid_case) {
      case 0: algorithm.mutable_label(1).Update(99, {35,35}, 35, {}, kInvalidRestriction); break;
      case 1: algorithm.mutable_label(0).Update(1, {15,15}, 15, {}, kInvalidRestriction); break;
      case 2: algorithm.mutable_label(0) = ExperimentalAlgorithm::label(3, kInvalidLabel); break;
      case 3: algorithm.origin.mutable_correlation()->clear_edges(); break;
      case 4: {
        const auto duplicate = algorithm.origin.correlation().edges(0);
        *algorithm.origin.mutable_correlation()->add_edges() = duplicate;
        break;
      }
      case 5: algorithm.mutable_label(1).set_destination(); break;
      case 6: algorithm.mutable_label(1) = ExperimentalAlgorithm::label(1, 0, Use::kRoad, true); break;
      case 7: algorithm.mutable_label(1) = ExperimentalAlgorithm::label(1, 0, Use::kTransitConnection); break;
      case 8: algorithm.mutable_label(1) = ExperimentalAlgorithm::label(1, 0, Use::kRoad, false, sif::TravelMode::kPedestrian); break;
    }
    const auto before = algorithm.labels();
    EXPECT_FALSE(algorithm.CopyExperimentalChain(algorithm.origin, algorithm.destination));
    for (size_t i = 0; i < before.size(); ++i)
      EXPECT_EQ(label_values(algorithm.labels()[i]), label_values(before[i]));
  }
}

TEST(ExperimentalChain, RejectsNonFiniteOriginFraction) {
  for (const float fraction : {std::numeric_limits<float>::quiet_NaN(),
                               std::numeric_limits<float>::infinity(),
                               -std::numeric_limits<float>::infinity()}) {
    ExperimentalAlgorithm algorithm;
    algorithm.seed();
    algorithm.origin.mutable_correlation()->mutable_edges(0)->set_percent_along(fraction);
    const auto before = algorithm.labels();
    const auto candidate = algorithm.candidate();
    EXPECT_FALSE(algorithm.CopyExperimentalChain(algorithm.origin, algorithm.destination));
    EXPECT_EQ(algorithm.candidate(), candidate);
    ASSERT_EQ(algorithm.labels().size(), before.size());
    for (size_t i = 0; i < before.size(); ++i)
      EXPECT_EQ(label_values(algorithm.labels()[i]), label_values(before[i]));
  }
}

TEST(ExperimentalChain, BoundAndAutonomousCopyAfterClear) {
  ExperimentalAlgorithm algorithm;
  algorithm.seed(4096);
  auto limit = algorithm.CopyExperimentalChain(algorithm.origin, algorithm.destination);
  ASSERT_TRUE(limit);
  EXPECT_EQ(limit->steps.size(), 4096);
  algorithm.seed(4097);
  EXPECT_FALSE(algorithm.CopyExperimentalChain(algorithm.origin, algorithm.destination));
  algorithm.seed();
  auto snapshot = algorithm.CopyExperimentalChain(algorithm.origin, algorithm.destination);
  ASSERT_TRUE(snapshot);
  const auto before = snapshot_values(*snapshot);
  const auto origin = snapshot->origin_edge.SerializeAsString();
  algorithm.Clear();
  EXPECT_EQ(algorithm.candidate(), kInvalidLabel);
  EXPECT_TRUE(algorithm.labels().empty());
  algorithm.seed(3, Use::kFerry);
  EXPECT_EQ(snapshot_values(*snapshot), before);
  EXPECT_EQ(snapshot->origin_edge.SerializeAsString(), origin);
}

class ExperimentalWorker : public thor_worker_t {
public:
  explicit ExperimentalWorker(const boost::property_tree::ptree& config,
                               const std::shared_ptr<GraphReader>& reader)
      : thor_worker_t(config, reader) {}
  void observe(TimeDepForward::experimental_chain_observer_t observer) {
    timedep_forward.SetExperimentalChainObserver(std::move(observer));
  }
  std::vector<std::vector<PathInfo>> run(Api& request) {
    const auto costing = parse_costing(request);
    timedep_forward.Clear();
    return get_path(&timedep_forward, *request.mutable_options()->mutable_locations(0),
                    *request.mutable_options()->mutable_locations(1), costing, request);
  }
  uint32_t pass() const { return mode_costing[static_cast<uint32_t>(mode)]->pass(); }
  bool ferry() const { return timedep_forward.has_ferry(); }
  void track(const PathAlgorithm::expansion_callback_t& callback) {
    timedep_forward.set_track_expansion(callback);
  }
  TimeDepForward& algorithm() { return timedep_forward; }
};

// Real engine over synthetic Gurka tiles; correlations are supplied manually.
// This bypasses Loki correlation and public HTTP serialization.
class ExperimentalCustody : public ::testing::Test {
protected:
  static gurka::map graph;
  static void SetUpTestSuite() {
    const auto layout = gurka::detail::map_to_coordinates("A---B---C D---E", 100);
    const gurka::ways ways = {
        {"AB", {{"highway","residential"},{"oneway","yes"}}},
        {"BC", {{"highway","residential"},{"oneway","yes"}}},
        {"DE", {{"highway","residential"},{"oneway","yes"}}}};
    graph = gurka::buildtiles(layout, ways, {}, {},
                              VALHALLA_BUILD_DIR "test/data/experimental_chain_custody");
  }
  static PathEdge correlation(GraphReader& reader, const std::string& way,
                               const std::string& end, double fraction) {
    const auto edge = std::get<0>(gurka::findEdge(reader, graph.nodes, way, end));
    const auto begin = graph.nodes.at(way.substr(0,1));
    const auto finish = graph.nodes.at(end);
    PathEdge result;
    result.set_graph_id(edge.value);
    result.set_percent_along(fraction);
    result.mutable_ll()->set_lng(begin.lng() + fraction * (finish.lng()-begin.lng()));
    result.mutable_ll()->set_lat(begin.lat() + fraction * (finish.lat()-begin.lat()));
    return result;
  }
  static Api request(GraphReader& reader, int kind) {
    Api request;
    ParseApi(R"({"locations":[{"lat":0,"lon":0},{"lat":0,"lon":0}],"costing":"auto","date_time":{"type":1,"value":"2026-10-08T10:01"}})", Options::route, request);
    auto* origin = request.mutable_options()->mutable_locations(0);
    const auto origin_edge = correlation(reader,"AB","B",.25);
    *origin->mutable_correlation()->add_edges() = origin_edge;
    *origin->mutable_ll() = origin_edge.ll();
    origin->set_date_time("2026-10-08T10:01");
    auto* destination = request.mutable_options()->mutable_locations(1);
    const auto ordinary_edge = correlation(reader,"BC","C",1);
    const auto disconnected_edge = correlation(reader,"DE","E",.5);
    const auto chosen = kind == 1 ? ordinary_edge : disconnected_edge;
    *destination->mutable_correlation()->add_edges() = chosen;
    *destination->mutable_ll() = chosen.ll();
    if (kind == 2)
      *destination->mutable_correlation()->add_filtered_edges() = ordinary_edge;
    return request;
  }
};
gurka::map ExperimentalCustody::graph;

TEST_F(ExperimentalCustody, OrdinaryEquivalenceAndFirstPassCustody) {
  for (const int kind : {0, 1, 2}) {
    auto reader_off = std::make_shared<GraphReader>(graph.config.get_child("mjolnir"));
    auto reader_on = std::make_shared<GraphReader>(graph.config.get_child("mjolnir"));
    ExperimentalWorker off(graph.config, reader_off), on(graph.config, reader_on);
    Api before = request(*reader_off,kind);
    Api observed = request(*reader_on,kind);
    std::vector<Snapshot> snapshots;
    using CallbackValue = std::tuple<uint64_t, uint64_t, std::string, int, float,
                                     uint32_t, float, int, uint8_t, int>;
    std::vector<CallbackValue> callbacks_off, callbacks_on;
    auto track = [](auto& sink) {
      return [&sink](GraphReader&, const GraphId& edge, const GraphId& pred,
                      const char* algorithm_name, Expansion_EdgeStatus status, float seconds,
                      uint32_t distance, float cost, Expansion_ExpansionType direction,
                      uint8_t flow_mask, valhalla::TravelMode travel_mode) {
        // Reader object identity differs by design; every payload field is retained.
        sink.emplace_back(edge.value, pred.value, std::string(algorithm_name),
                          static_cast<int>(status), seconds, distance, cost,
                          static_cast<int>(direction), flow_mask, static_cast<int>(travel_mode));
      };
    };
    off.track(track(callbacks_off));
    on.track(track(callbacks_on));
    on.observe([&](Snapshot&& snapshot) { snapshots.push_back(std::move(snapshot)); });
    const auto paths_off = off.run(before);
    const auto paths_on = on.run(observed);
    expect_same_paths(paths_off,paths_on);
    EXPECT_EQ(before.SerializeAsString(), observed.SerializeAsString());
    EXPECT_EQ(off.pass(),on.pass());
    EXPECT_EQ(off.ferry(),on.ferry());
    EXPECT_EQ(callbacks_off,callbacks_on);
    if (kind == 0) {
      EXPECT_TRUE(paths_on.empty());
      ASSERT_EQ(snapshots.size(),1);
      const auto& snapshot = snapshots.front();
      ASSERT_EQ(snapshot.steps.size(),2);
      EXPECT_EQ(snapshot.origin_edge.percent_along(),.25);
      // Independent oracle: stored directed-edge length and the frozen .25 origin.
      // SetOrigin must actually traverse 75% of AB; manual costs 15/35 are unrelated.
      const auto ab = gurka::findEdge(*reader_on, graph.nodes, "AB", "B");
      ASSERT_NE(std::get<1>(ab), nullptr);
      const auto expected_ab_distance =
          static_cast<uint32_t>(std::get<1>(ab)->length() * .75f + .5f);
      EXPECT_EQ(snapshot.steps.front().label.path_distance(), expected_ab_distance);
      EXPECT_EQ(snapshot.steps.front().label.edgeid(),std::get<0>(gurka::findEdge(*reader_on,graph.nodes,"AB","B")));
      EXPECT_EQ(snapshot.steps.back().label.edgeid(),std::get<0>(gurka::findEdge(*reader_on,graph.nodes,"BC","C")));
      EXPECT_EQ(snapshot.steps.front().label.endnode(),gurka::findNode(*reader_on,graph.nodes,"B"));
      EXPECT_EQ(snapshot.steps.back().label.endnode(),gurka::findNode(*reader_on,graph.nodes,"C"));
      EXPECT_EQ(snapshot.steps.back().label.predecessor(),snapshot.steps.front().label_index);
      EXPECT_EQ(snapshot.pass,0);
      for (const auto& step : snapshot.steps) {
        EXPECT_FALSE(step.label.destination());
        EXPECT_FALSE(step.label.shortcut());
        EXPECT_EQ(step.label.mode(),sif::TravelMode::kDrive);
        const auto predecessor = step.label.predecessor() == kInvalidLabel
                                     ? GraphId{}.value
                                     : snapshot.steps.front().label.edgeid().value;
        const auto event = std::find_if(callbacks_on.begin(), callbacks_on.end(),
            [&](const CallbackValue& value) {
              return std::get<0>(value) == step.label.edgeid().value &&
                     std::get<1>(value) == predecessor &&
                     std::get<3>(value) == static_cast<int>(Expansion_EdgeStatus_settled) &&
                     std::get<7>(value) == static_cast<int>(Expansion_ExpansionType_forward);
            });
        EXPECT_NE(event, callbacks_on.end());
      }
    } else {
      EXPECT_FALSE(paths_on.empty());
      EXPECT_TRUE(snapshots.empty());
      EXPECT_EQ(on.pass(),kind == 2 ? 1 : 0);
    }
  }
}

TEST_F(ExperimentalCustody, ResetCleanupAndInterruptionDoNotPublishStaleData) {
  auto reader = std::make_shared<GraphReader>(graph.config.get_child("mjolnir"));
  ExperimentalWorker worker(graph.config, reader);
  std::vector<Snapshot> snapshots;
  worker.observe([&](Snapshot&& value) { snapshots.push_back(std::move(value)); });
  auto fail = request(*reader,0);
  EXPECT_TRUE(worker.run(fail).empty());
  ASSERT_EQ(snapshots.size(),1);
  const auto frozen = snapshot_values(snapshots.front());
  worker.cleanup(); // Assert before the next run/Clear.
  EXPECT_FALSE(worker.algorithm().CopyExperimentalChain(
      fail.options().locations(0), fail.options().locations(1)));
  auto success = request(*reader,1);
  EXPECT_FALSE(worker.run(success).empty());
  EXPECT_EQ(snapshots.size(),1);
  EXPECT_EQ(snapshot_values(snapshots.front()),frozen);
  worker.cleanup(); // Assert before the next run/Clear.
  EXPECT_FALSE(worker.algorithm().CopyExperimentalChain(
      success.options().locations(0), success.options().locations(1)));
  bool interrupted = false;
  worker.track([&](GraphReader&, const GraphId&, const GraphId&, const char*,
                    Expansion_EdgeStatus status, float, uint32_t, float,
                    Expansion_ExpansionType, uint8_t, valhalla::TravelMode) {
    if (status == Expansion_EdgeStatus_settled) {
      interrupted = true;
      throw std::runtime_error("fixture interruption");
    }
  });
  auto next = request(*reader,0);
  EXPECT_THROW(worker.run(next),std::runtime_error);
  EXPECT_TRUE(interrupted);
  EXPECT_EQ(snapshots.size(),1);
  worker.cleanup(); // Assert before the next run/Clear.
  EXPECT_FALSE(worker.algorithm().CopyExperimentalChain(
      next.options().locations(0), next.options().locations(1)));
  worker.track({});
  auto clean = request(*reader,1);
  EXPECT_FALSE(worker.run(clean).empty());
  EXPECT_EQ(snapshots.size(),1);
}

TEST_F(ExperimentalCustody, ObserverFailureCannotChangeOrdinaryFailure) {
  auto reader = std::make_shared<GraphReader>(graph.config.get_child("mjolnir"));
  ExperimentalWorker worker(graph.config,reader);
  unsigned calls = 0;
  worker.observe([&](Snapshot&&) { ++calls; throw std::runtime_error("observer"); });
  auto fail = request(*reader,0);
  EXPECT_NO_THROW(EXPECT_TRUE(worker.run(fail).empty()));
  EXPECT_EQ(calls,1);
}

TEST_F(ExperimentalCustody, ExcludedProfilesHaveNoCandidate) {
  auto reader = std::make_shared<GraphReader>(graph.config.get_child("mjolnir"));
  unsigned calls = 0;
  TimeDepForward algorithm;
  algorithm.SetExperimentalChainObserver([&](Snapshot&&) { ++calls; });
  for (int excluded = 0; excluded < 2; ++excluded) {
    auto api = request(*reader,0);
    if (excluded == 0) {
      api.mutable_options()->set_costing_type(Costing::pedestrian);
      rapidjson::Document costing_options;
      costing_options.SetObject();
      sif::ParseCosting(costing_options, "/costing_options", *api.mutable_options(),
                       *api.mutable_info()->mutable_warnings());
    } else
      api.mutable_options()->mutable_locations(0)->clear_date_time();
    sif::TravelMode mode;
    auto costs = CostFactory().CreateModeCosting(api.options(),mode);
    algorithm.Clear();
    auto origin = api.options().locations(0), destination = api.options().locations(1);
    algorithm.GetBestPath(origin,destination,*reader,costs,mode,api.options());
    EXPECT_FALSE(algorithm.CopyExperimentalChain(origin,destination));
  }
  TimeDepReverse reverse;
  unsigned reverse_calls = 0;
  reverse.SetExperimentalChainObserver([&](TimeDepReverse::ExperimentalChainSnapshot&&) { ++reverse_calls; });
  auto api = request(*reader,0);
  auto origin = api.options().locations(0), destination = api.options().locations(1);
  destination.set_date_time("2026-10-08T10:01");
  sif::TravelMode mode;
  auto costs = CostFactory().CreateModeCosting(api.options(),mode);
  reverse.GetBestPath(origin,destination,*reader,costs,mode,api.options());
  EXPECT_FALSE(reverse.CopyExperimentalChain(origin,destination));
  EXPECT_EQ(calls,0);
  EXPECT_EQ(reverse_calls,0);
}

// Offline Torino custody gate: real Loki correlations, no HTTP or route serialization.
class TorinoCustodyWorker : public ExperimentalWorker {
public:
  using ExperimentalWorker::ExperimentalWorker;
  std::vector<std::vector<PathInfo>> run_correlated(Api& api) {
    adjust_locations(api);
    const auto costing = parse_costing(api);
    auto& origin = *api.mutable_options()->mutable_locations(0);
    auto& destination = *api.mutable_options()->mutable_locations(1);
    auto* selected = get_path_algorithm(costing, origin, destination, api);
    if (selected != &timedep_forward)
      throw std::runtime_error("Torino gate: ordinary selector did not choose TimeDepForward");
    return get_path(selected, origin, destination, costing, api);
  }
};

// Opt-in fixture only: disabled in unfiltered runs; explicitly filtered for the Gate.
TEST(TorinoCustody, DISABLED_PapacinoFrozenCases) {
  boost::property_tree::ptree config;
  boost::property_tree::read_json(
      "C:/Users/renat/Documents/Lumoflutter/valhalla_phase2a_bench/valhalla.json", config);
  for (const auto* key : {"tile_extract", "traffic_extract", "tile_url", "incident_log",
                          "incident_tile_dir", "landmarks"})
    config.get_child("mjolnir").erase(key);
  config.put("logging.type", "std_out");
  config.get_child("logging").erase("file_name");
  using Event = std::tuple<uint64_t, uint64_t, std::string, int, float,
                           uint32_t, float, int, uint8_t, int>;
  const auto track = [](std::vector<Event>& events) {
    return [&events](GraphReader&, const GraphId& edge, const GraphId& pred,
                     const char* name, Expansion_EdgeStatus status, float seconds,
                     uint32_t distance, float cost, Expansion_ExpansionType direction,
                     uint8_t flow, valhalla::TravelMode mode) {
      events.emplace_back(edge.value, pred.value, std::string(name), int(status), seconds,
                          distance, cost, int(direction), flow, int(mode));
    };
  };
  struct Case { const char* time; bool historical_arrival; bool ordinary_success; };
  for (const auto& test : {Case{"06:30", false, true}, Case{"10:01", false, false},
                           Case{"11:00", false, true}, Case{"10:01", true, true}}) {
    SCOPED_TRACE(std::string(test.time) + (test.historical_arrival ? " arrival" : " Papacino"));
    const double lat = test.historical_arrival ? 45.0678875 : 45.0687289;
    const double lon = test.historical_arrival ? 7.672596100000001 : 7.6720592;
    const std::string input = std::string(R"({"locations":[{"lat":45.0673126,"lon":7.7012488},{"lat":)") +
        (test.historical_arrival ? "45.0678875,\"lon\":7.672596100000001" :
                                  "45.0687289,\"lon\":7.6720592") +
        R"(}],"costing":"auto","costing_options":{"auto":{}},"date_time":{"type":1,"value":"2026-10-08T)" +
        test.time + R"("},"language":"it-IT","units":"kilometers","shape_format":"polyline6"})";
    auto reader_off = std::make_shared<GraphReader>(config.get_child("mjolnir"));
    auto reader_on = std::make_shared<GraphReader>(config.get_child("mjolnir"));
    loki_worker_t loki(config, reader_off);
    Api correlated;
    ParseApi(input, Options::route, correlated);
    loki.route(correlated);
    loki.cleanup();
    ASSERT_EQ(correlated.options().locations_size(), 2);
    ASSERT_FALSE(correlated.options().locations(0).correlation().edges().empty());
    ASSERT_FALSE(correlated.options().locations(1).correlation().edges().empty());
    ASSERT_EQ(correlated.options().locations(0).date_time(),
              std::string("2026-10-08T") + test.time);
    Api off_api = correlated, on_api = correlated;
    TorinoCustodyWorker off(config, reader_off), on(config, reader_on);
    std::vector<Event> off_events, on_events;
    std::vector<Snapshot> snapshots;
    off.track(track(off_events)); on.track(track(on_events));
    on.observe([&](Snapshot&& value) { snapshots.push_back(std::move(value)); });
    const auto off_paths = off.run_correlated(off_api);
    const auto on_paths = on.run_correlated(on_api);
    ASSERT_EQ(!off_paths.empty(), test.ordinary_success);
    expect_same_paths(off_paths, on_paths);
    ASSERT_FALSE(HasFailure());
    ASSERT_EQ(off_events, on_events);
    ASSERT_EQ(off.pass(), on.pass());
    ASSERT_EQ(off.ferry(), on.ferry());
    ASSERT_EQ(off.algorithm().not_thru_pruning(), on.algorithm().not_thru_pruning());
    ASSERT_FALSE(off.algorithm().HasExperimentalChainObserver());
    ASSERT_TRUE(on.algorithm().HasExperimentalChainObserver());
    ASSERT_EQ(off_api.options().SerializeAsString(), on_api.options().SerializeAsString());
    ASSERT_EQ(off_api.info().warnings_size(), on_api.info().warnings_size());
    for (int i = 0; i < off_api.info().warnings_size(); ++i)
      ASSERT_EQ(off_api.info().warnings(i).SerializeAsString(), on_api.info().warnings(i).SerializeAsString());
    ASSERT_DOUBLE_EQ(on_api.options().locations(1).ll().lat(), lat);
    ASSERT_DOUBLE_EQ(on_api.options().locations(1).ll().lng(), lon);
    if (test.ordinary_success) {
      ASSERT_TRUE(snapshots.empty());
    } else {
      ASSERT_EQ(on.pass(), 1);
      ASSERT_EQ(snapshots.size(), 1);
      const auto& snapshot = snapshots.front();
      ASSERT_FALSE(snapshot.steps.empty());
      ASSERT_LE(snapshot.steps.size(), TimeDepForward::kExperimentalChainLimit);
      ASSERT_EQ(snapshot.pass, 0);
      ASSERT_DOUBLE_EQ(snapshot.requested_destination.lat(), lat);
      ASSERT_DOUBLE_EQ(snapshot.requested_destination.lng(), lon);
      ASSERT_EQ(snapshot.origin_edge.graph_id(), snapshot.steps.front().label.edgeid().value);
      ASSERT_TRUE(std::isfinite(snapshot.origin_edge.percent_along()));
      ASSERT_GE(snapshot.origin_edge.percent_along(), 0);
      ASSERT_LE(snapshot.origin_edge.percent_along(), 1);
      ASSERT_TRUE(snapshot.steps.front().label.origin());
      ASSERT_EQ(snapshot.steps.front().label.predecessor(), kInvalidLabel);
      const auto& roots = on_api.options().locations(0).correlation().edges();
      const auto root = std::find_if(roots.begin(), roots.end(), [&](const PathEdge& e) {
        return e.graph_id() == snapshot.origin_edge.graph_id();
      });
      ASSERT_NE(root, roots.end());
      ASSERT_EQ(root->SerializeAsString(), snapshot.origin_edge.SerializeAsString());
      for (size_t i = 0; i < snapshot.steps.size(); ++i) {
        const auto& step = snapshot.steps[i];
        ASSERT_EQ(step.label.mode(), sif::TravelMode::kDrive);
        ASSERT_FALSE(step.label.destination());
        ASSERT_FALSE(step.label.shortcut());
        if (i) ASSERT_EQ(step.label.predecessor(), snapshot.steps[i-1].label_index);
        const auto event = std::find_if(on_events.begin(), on_events.end(), [&](const Event& e) {
          return std::get<0>(e) == step.label.edgeid().value &&
                 std::get<1>(e) == (i ? snapshot.steps[i-1].label.edgeid().value : GraphId{}.value) &&
                 std::get<3>(e) == int(Expansion_EdgeStatus_settled) &&
                 std::get<7>(e) == int(Expansion_ExpansionType_forward);
        });
        ASSERT_NE(event, on_events.end());
        const auto tile = reader_on->GetGraphTile(step.label.edgeid());
        ASSERT_TRUE(tile);
        ASSERT_EQ(tile->directededge(step.label.edgeid())->endnode(), step.label.endnode());
      }
      const auto tile = reader_on->GetGraphTile(snapshot.steps.back().label.endnode());
      ASSERT_TRUE(tile);
      ASSERT_EQ(snapshot.arrival, tile->get_node_ll(snapshot.steps.back().label.endnode()));
    }
    std::cout << std::setprecision(15) << "TORINO_CASE " << test.time << " historical_arrival=" << test.historical_arrival
              << " ordinary_success=" << !on_paths.empty() << " pass=" << on.pass()
              << " callbacks=" << on_events.size() << " snapshots=" << snapshots.size() << std::endl;
    for (const auto& s : snapshots)
      std::cout << "TORINO_SNAPSHOT pass=" << s.pass << " labels=" << s.steps.size()
                << " arrival=" << s.arrival.lat() << "," << s.arrival.lng() << std::endl;
    for (const auto& s : snapshots) {
      std::cout << "TORINO_ORIGIN edge=" << s.origin_edge.graph_id()
                << " fraction=" << s.origin_edge.percent_along() << std::endl;
      for (const auto& step : s.steps)
        std::cout << "TORINO_LABEL index=" << step.label_index
                  << " predecessor=" << step.label.predecessor()
                  << " edge=" << step.label.edgeid().value
                  << " endnode=" << step.label.endnode().value
                  << " cost=" << step.label.cost().cost
                  << " seconds=" << step.label.cost().secs << std::endl;
    }
    std::vector<decltype(snapshot_values(Snapshot{}))> values;
    std::vector<std::string> origins;
    std::vector<std::tuple<double,double,double,double,uint32_t>> metadata;
    for (const auto& s : snapshots)
      metadata.emplace_back(s.arrival.lat(), s.arrival.lng(), s.requested_destination.lat(),
                            s.requested_destination.lng(), s.pass);
    for (const auto& s : snapshots) { values.push_back(snapshot_values(s)); origins.push_back(s.origin_edge.SerializeAsString()); }
    off.cleanup(); on.cleanup(); // Inspect immediately, before another Clear can mask cleanup.
    ASSERT_FALSE(on.algorithm().CopyExperimentalChain(on_api.options().locations(0), on_api.options().locations(1)));
    ASSERT_FALSE(off.algorithm().CopyExperimentalChain(off_api.options().locations(0), off_api.options().locations(1)));
    ASSERT_EQ(off.pass(), on.pass()); ASSERT_EQ(off.ferry(), on.ferry());
    ASSERT_EQ(off.algorithm().not_thru_pruning(), on.algorithm().not_thru_pruning());
    on.algorithm().Clear();
    for (size_t i = 0; i < snapshots.size(); ++i) {
      const auto& s = snapshots[i];
      ASSERT_EQ(std::make_tuple(s.arrival.lat(), s.arrival.lng(), s.requested_destination.lat(),
                                s.requested_destination.lng(), s.pass), metadata[i]);
      ASSERT_EQ(snapshot_values(snapshots[i]), values[i]);
      ASSERT_EQ(snapshots[i].origin_edge.SerializeAsString(), origins[i]);
    }
  }
}


// Frozen before implementation: original Papacino at 06:30/11:00 succeeds with
// zero confirmations; 10:01 fails with 442, yields 94 labels, and confirms once.
// This traverses real Loki + Thor and the same helper used by HTTP work().
// It does not certify HTTP transport, Lumo adoption/panel or Android/iOS.
class PapacinoConfirmationWorker : public thor_worker_t {
public:
  using thor_worker_t::thor_worker_t;
  bool observing() const { return timedep_forward.HasExperimentalChainObserver(); }
  void track(const PathAlgorithm::expansion_callback_t& callback) {
    timedep_forward.set_track_expansion(callback);
  }
};

TEST(PapacinoConfirmation, DISABLED_FrozenCasesAndNextRequest) {
  boost::property_tree::ptree config;
  boost::property_tree::read_json(
      "C:/Users/renat/Documents/Lumoflutter/valhalla_phase2a_bench/valhalla.json", config);
  for (const auto* key : {"tile_extract", "traffic_extract", "tile_url", "incident_log",
                          "incident_tile_dir", "landmarks"})
    config.get_child("mjolnir").erase(key);
  config.put("logging.type", "std_out");
  config.get_child("logging").erase("file_name");
  ASSERT_FALSE(config.get<bool>("thor.experimental_custodied_arrival", false));
  auto enabled = config;
  enabled.put("thor.experimental_custodied_arrival", true);
  auto off_reader = std::make_shared<GraphReader>(config.get_child("mjolnir"));
  auto on_reader = std::make_shared<GraphReader>(config.get_child("mjolnir"));
  PapacinoConfirmationWorker off(config, off_reader), on(enabled, on_reader);
  const std::function<void()> interrupt_function = []() {};
  using Event = std::tuple<uint64_t, uint64_t, std::string, int, float,
                           uint32_t, float, int, uint8_t, int>;
  auto track = [](std::vector<Event>& events) {
    return [&events](GraphReader&, const GraphId& edge, const GraphId& predecessor,
                     const char* name, Expansion_EdgeStatus status, float seconds,
                     uint32_t distance, float cost, Expansion_ExpansionType direction,
                     uint8_t flow, valhalla::TravelMode mode) {
      events.emplace_back(edge.value, predecessor.value, std::string(name), int(status),
                          seconds, distance, cost, int(direction), flow, int(mode));
    };
  };
  // Failure first, then successes on the same worker: catches stale custody.
  for (const auto* time : {"10:01", "06:30", "11:00"}) {
    SCOPED_TRACE(time);
    const bool alternative = std::string(time) == "10:01";
    const std::string input =
        std::string(R"({"locations":[{"lat":45.0673126,"lon":7.7012488},{"lat":45.0687289,"lon":7.6720592}],"costing":"auto","costing_options":{"auto":{}},"date_time":{"type":1,"value":"2026-10-08T)") +
        time + R"("},"language":"it-IT","units":"kilometers","shape_format":"polyline6"})";
    Api correlated;
    ParseApi(input, Options::route, correlated);
    loki_worker_t loki(config, off_reader);
    loki.route(correlated);
    loki.cleanup();
    const auto original = correlated.options();
    Api off_api = correlated, on_api = correlated;
    std::vector<Event> off_events, on_events;
    auto detach = make_finally([&]() {
      off.track({}); on.track({});
      off.cleanup(); on.cleanup();
      off.set_interrupt(nullptr); on.set_interrupt(nullptr);
    });
    off.track(track(off_events)); on.track(track(on_events));
    off.set_interrupt(&interrupt_function); on.set_interrupt(&interrupt_function);
    unsigned off_code = 0;
    try {
      const auto ordinary = off.route_with_custodied_arrival(off_api, interrupt_function);
      ASSERT_TRUE(ordinary.ordinary);
      ASSERT_EQ(ordinary.confirmation_attempts, 0);
    } catch (const valhalla_exception_t& error) {
      off_code = error.code;
    }
    ASSERT_EQ(off_code, alternative ? 442u : 0u);
    auto actual = on.route_with_custodied_arrival(on_api, interrupt_function);
    ASSERT_EQ(actual.ordinary, !alternative);
    ASSERT_EQ(actual.confirmation_attempts, alternative ? 1u : 0u);
    ASSERT_EQ(off_events, on_events); // Original route only: fresh actor is unobserved.
    ASSERT_EQ(off_api.options().SerializeAsString(), on_api.options().SerializeAsString());
    ASSERT_EQ(off_api.trip().SerializeAsString(), on_api.trip().SerializeAsString());
    ASSERT_EQ(off_api.info().warnings_size(), on_api.info().warnings_size());
    for (int i = 0; i < off_api.info().warnings_size(); ++i)
      ASSERT_EQ(off_api.info().warnings(i).SerializeAsString(),
                on_api.info().warnings(i).SerializeAsString());
    ASSERT_FALSE(on.observing());
    ASSERT_FALSE(off.observing());
    ASSERT_DOUBLE_EQ(on_api.options().locations(1).ll().lat(), 45.0687289);
    ASSERT_DOUBLE_EQ(on_api.options().locations(1).ll().lng(), 7.6720592);
    if (alternative) {
      ASSERT_EQ(actual.custody_labels, 94);
      ASSERT_NEAR(actual.candidate.lat(), 45.0678875, 1e-7);
      ASSERT_NEAR(actual.candidate.lng(), 7.672596100000001, 1e-7);
      const auto& confirmed = actual.confirmation;
      ASSERT_EQ(confirmed.options().costing_type(), original.costing_type());
      ASSERT_EQ(confirmed.options().date_time_type(), original.date_time_type());
      ASSERT_EQ(confirmed.options().date_time(), original.date_time());
      ASSERT_EQ(confirmed.options().costings().size(), original.costings().size());
      for (const auto& costing : original.costings()) {
        const auto found = confirmed.options().costings().find(costing.first);
        ASSERT_NE(found, confirmed.options().costings().end());
        ASSERT_EQ(found->second.SerializeAsString(), costing.second.SerializeAsString());
      }
      ASSERT_DOUBLE_EQ(confirmed.options().locations(0).ll().lat(), original.locations(0).ll().lat());
      ASSERT_DOUBLE_EQ(confirmed.options().locations(0).ll().lng(), original.locations(0).ll().lng());
      ASSERT_DOUBLE_EQ(confirmed.options().locations(1).ll().lat(), actual.candidate.lat());
      ASSERT_DOUBLE_EQ(confirmed.options().locations(1).ll().lng(), actual.candidate.lng());
      ASSERT_EQ(confirmed.trip().routes_size(), 1);
      ASSERT_EQ(confirmed.trip().routes(0).legs_size(), 1);
      const auto shape =
          midgard::decode<std::vector<PointLL>>(confirmed.trip().routes(0).legs(0).shape());
      ASSERT_GE(shape.size(), 2);
      // One polyline6 unit: representation precision, not an arrival search radius.
      ASSERT_NEAR(shape.back().lat(), actual.candidate.lat(), 1e-6);
      ASSERT_NEAR(shape.back().lng(), actual.candidate.lng(), 1e-6);
      ASSERT_EQ(confirmed.options().locations(0).date_time(), original.date_time());
      ASSERT_EQ(confirmed.options().locations(0).time_zone_name(), "Europe/Rome");
      const PointLL destination{7.6720592, 45.0687289};
      const auto residual = shape.back().Distance(destination);
      rapidjson::Document body;
      body.Parse(actual.response.c_str());
      ASSERT_FALSE(body.HasParseError());
      ASSERT_TRUE(body.IsObject());
      ASSERT_TRUE(body.HasMember("trip"));
      ASSERT_TRUE(body["trip"].IsObject());
      ASSERT_TRUE(body["trip"].HasMember("status"));
      ASSERT_TRUE(body["trip"]["status"].IsInt());
      ASSERT_EQ(body["trip"]["status"].GetInt(), 0);
      ASSERT_TRUE(body["trip"].HasMember("summary"));
      ASSERT_TRUE(body["trip"]["summary"].IsObject());
      ASSERT_TRUE(body["trip"]["summary"].HasMember("length"));
      ASSERT_TRUE(body["trip"]["summary"]["length"].IsNumber());
      ASSERT_TRUE(body["trip"]["summary"].HasMember("time"));
      ASSERT_TRUE(body["trip"]["summary"]["time"].IsNumber());
      ASSERT_GT(body["trip"]["summary"]["length"].GetDouble(), 0);
      ASSERT_GT(body["trip"]["summary"]["time"].GetDouble(), 0);
      std::cout << "PAPACINO_CONFIRMATION " << time << " count=1 labels="
                << actual.custody_labels << " length_km="
                << body["trip"]["summary"]["length"].GetDouble() << " seconds="
                << body["trip"]["summary"]["time"].GetDouble()
                << " residual_m=" << residual << std::endl;
    } else {
      ASSERT_EQ(actual.custody_labels, 0);
      ASSERT_TRUE(actual.response.empty());
      odin::odin_worker_t off_odin(config), on_odin(config);
      ASSERT_EQ(off_odin.narrate(off_api), on_odin.narrate(on_api));
    }
    off.track({}); on.track({});
    off.cleanup(); on.cleanup();
    ASSERT_FALSE(on.observing());
    off.set_interrupt(nullptr); on.set_interrupt(nullptr);
    ASSERT_FALSE(HasFailure());
  }
}

} // namespace

int main(int argc, char* argv[]) {
  // logging::Configure({{"type", ""}}); // silence logs
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
