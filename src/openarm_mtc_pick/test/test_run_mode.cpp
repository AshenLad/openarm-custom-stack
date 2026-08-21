// Unit tests for the stage-latch run mode model (run_mode.hpp).
//
// Pins the §12 pipeline of every run_mode, the §14 terminal guarantee, and
// the execution latch (plan_only never executes; full needs the extra
// allow_full_execution flag).

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "run_mode.hpp"

namespace openarm_mtc {
namespace {

bool plan_contains(const std::vector<StageKind>& plan, const StageKind kind) {
  return std::find(plan.begin(), plan.end(), kind) != plan.end();
}

std::vector<StageKind> pick_and_place_plan() {
  return {
      StageKind::CurrentState,
      StageKind::OpenGripper,
      StageKind::ConnectPick,
      StageKind::Approach,
      StageKind::GraspIK,
      StageKind::AllowHandObject,
      StageKind::Close,
      StageKind::AllowObjectTable,
      StageKind::Attach,
      StageKind::Lift,
      StageKind::ForbidObjectTable,
      StageKind::AllowPlaceSupport,
      StageKind::ConnectPlace,
      StageKind::PlaceLower,
      StageKind::PlaceIK,
      StageKind::PlaceOpen,
      StageKind::PlaceDetach,
      StageKind::ForbidHandObject,
      StageKind::ForbidPlaceSupport,
      StageKind::PlaceRetreat,
  };
}

}  // namespace

TEST(RunModeParse, AcceptsAllEightValuesAndRoundTrips) {
  const std::vector<std::string> names = {
      "plan_only", "open_only", "pregrasp_test", "approach_test",
      "close_test", "lift_test", "place_test", "full"};
  for (const auto& name : names) {
    EXPECT_EQ(run_mode_name(parse_run_mode(name)), name);
  }
}

TEST(RunModeParse, RejectsUnknownValues) {
  for (const auto& bad :
       {"", "full_pick", "pick", "FULL", "plan-only", "execute"}) {
    EXPECT_THROW(parse_run_mode(bad), std::invalid_argument);
  }
}

TEST(ExecutionLatch, PlanOnlyNeverExecutes) {
  EXPECT_FALSE(can_execute(RunMode::PlanOnly, true, true, true));
  EXPECT_FALSE(can_execute(RunMode::PlanOnly, false, false, false));
}

TEST(ExecutionLatch, StageModesNeedExecuteAndAllowExecution) {
  for (const RunMode mode : {RunMode::OpenOnly, RunMode::PregraspTest,
                             RunMode::ApproachTest, RunMode::CloseTest,
                             RunMode::LiftTest, RunMode::PlaceTest}) {
    EXPECT_FALSE(can_execute(mode, false, false, false));
    EXPECT_FALSE(can_execute(mode, true, false, false));
    EXPECT_FALSE(can_execute(mode, false, true, false));
    EXPECT_TRUE(can_execute(mode, true, true, false))
        << "stage mode " << run_mode_name(mode)
        << " must not require allow_full_execution";
  }
}

TEST(ExecutionLatch, FullRequiresAllowFullExecution) {
  EXPECT_FALSE(can_execute(RunMode::Full, true, true, false));
  EXPECT_TRUE(can_execute(RunMode::Full, true, true, true));
  EXPECT_FALSE(can_execute(RunMode::Full, true, false, true));
}

TEST(StagePlans, OpenOnlyStopsAfterOpen) {
  const auto plan = stage_plan_for_mode(RunMode::OpenOnly);
  EXPECT_EQ(plan, (std::vector<StageKind>{StageKind::CurrentState,
                                          StageKind::OpenGripper}));
}

TEST(StagePlans, PregraspTestStopsAfterPreGrasp) {
  const auto plan = stage_plan_for_mode(RunMode::PregraspTest);
  EXPECT_EQ(plan, (std::vector<StageKind>{StageKind::CurrentState,
                                          StageKind::OpenGripper,
                                          StageKind::PreGrasp}));
  EXPECT_FALSE(plan_contains(plan, StageKind::Approach));
  EXPECT_FALSE(plan_contains(plan, StageKind::Close));
}

TEST(StagePlans, ApproachTestStopsBeforeClose) {
  const auto plan = stage_plan_for_mode(RunMode::ApproachTest);
  EXPECT_EQ(plan, (std::vector<StageKind>{StageKind::CurrentState,
                                          StageKind::OpenGripper,
                                          StageKind::ConnectPick,
                                          StageKind::Approach,
                                          StageKind::GraspIK}));
  EXPECT_FALSE(plan_contains(plan, StageKind::Close));
  EXPECT_FALSE(plan_contains(plan, StageKind::Attach));
  EXPECT_FALSE(plan_contains(plan, StageKind::Lift));
  EXPECT_FALSE(plan_contains(plan, StageKind::PlaceIK));
  EXPECT_FALSE(plan_contains(plan, StageKind::ReturnInitial));
}

TEST(StagePlans, CloseTestStopsBeforeAttachLift) {
  const auto plan = stage_plan_for_mode(RunMode::CloseTest);
  EXPECT_EQ(plan, (std::vector<StageKind>{
                      StageKind::CurrentState, StageKind::OpenGripper,
                      StageKind::ConnectPick, StageKind::Approach,
                      StageKind::GraspIK, StageKind::AllowHandObject,
                      StageKind::Close}));
  EXPECT_FALSE(plan_contains(plan, StageKind::Attach));
  EXPECT_FALSE(plan_contains(plan, StageKind::Lift));
  EXPECT_FALSE(plan_contains(plan, StageKind::PlaceIK));
  EXPECT_FALSE(plan_contains(plan, StageKind::ReturnInitial));
}

TEST(StagePlans, LiftTestLandsBackWithoutReturnInitial) {
  const auto plan = stage_plan_for_mode(RunMode::LiftTest);
  const auto expected = pick_and_place_plan();
  ASSERT_EQ(plan.size(), expected.size());
  EXPECT_EQ(plan, expected);
  // The semantic difference vs place_test: lift_test must land back on the
  // measured grasp XY so the object does not move.
  EXPECT_TRUE(extent_is_lift_test(pipeline_extent_for_mode(RunMode::LiftTest)));
  EXPECT_FALSE(extent_is_lift_test(
      pipeline_extent_for_mode(RunMode::PlaceTest)));
  EXPECT_FALSE(plan_contains(plan, StageKind::ReturnInitial));
  EXPECT_EQ(plan.back(), StageKind::PlaceRetreat);
}

TEST(StagePlans, PlaceTestStopsAfterRetreatWithoutReturn) {
  const auto plan = stage_plan_for_mode(RunMode::PlaceTest);
  EXPECT_EQ(plan, pick_and_place_plan());
  EXPECT_FALSE(plan_contains(plan, StageKind::ReturnInitial));
  EXPECT_EQ(plan.back(), StageKind::PlaceRetreat);
}

TEST(StagePlans, FullAndPlanOnlyReturnInitial) {
  for (const RunMode mode : {RunMode::Full, RunMode::PlanOnly}) {
    const auto plan = stage_plan_for_mode(mode);
    const auto expected = pick_and_place_plan();
    ASSERT_EQ(plan.size(), expected.size() + 2);
    EXPECT_EQ(std::vector<StageKind>(plan.begin(), plan.begin() +
                                                      static_cast<long>(expected.size())),
              expected);
    EXPECT_EQ(plan[expected.size()], StageKind::PostReleaseClose);
    EXPECT_EQ(plan.back(), StageKind::ReturnInitial);
  }
}

TEST(StagePlans, EveryModeHasAnExplicitTerminalStop) {
  const std::vector<std::pair<RunMode, StageKind>> stops = {
      {RunMode::OpenOnly, StageKind::OpenGripper},
      {RunMode::PregraspTest, StageKind::PreGrasp},
      {RunMode::ApproachTest, StageKind::GraspIK},
      {RunMode::CloseTest, StageKind::Close},
      {RunMode::LiftTest, StageKind::PlaceRetreat},
      {RunMode::PlaceTest, StageKind::PlaceRetreat},
      {RunMode::Full, StageKind::ReturnInitial},
      {RunMode::PlanOnly, StageKind::ReturnInitial},
  };
  for (const auto& [mode, stop] : stops) {
    const auto plan = stage_plan_for_mode(mode);
    ASSERT_FALSE(plan.empty());
    EXPECT_EQ(plan.back(), stop)
        << "run_mode " << run_mode_name(mode)
        << " must stop at the §12 stage and never continue";
  }
}

TEST(TerminalMessages, EveryModePrintsADistinctTestCompleteLine) {
  std::vector<std::string> messages;
  for (const RunMode mode : {RunMode::PlanOnly, RunMode::OpenOnly,
                             RunMode::PregraspTest, RunMode::ApproachTest,
                             RunMode::CloseTest, RunMode::LiftTest,
                             RunMode::PlaceTest, RunMode::Full}) {
    const std::string message = test_complete_message(mode);
    EXPECT_FALSE(message.empty());
    EXPECT_NE(std::string::npos, message.find("COMPLETE"))
        << "message for " << run_mode_name(mode) << ": " << message;
    messages.push_back(message);
  }
  // Every message must be distinct so the operator can unambiguously tell
  // which stage latch fired.
  for (std::size_t i = 0; i < messages.size(); ++i) {
    for (std::size_t j = i + 1; j < messages.size(); ++j) {
      EXPECT_NE(messages[i], messages[j]);
    }
  }
}

}  // namespace openarm_mtc
