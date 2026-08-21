// Stage-latch run mode model (OpenArm MTC). Pure logic shared by the MTC
// executable and its unit tests so the constructed pipeline and the tested
// abstract plan cannot drift apart.
//
// §12 run_mode interface, §14 per-mode terminal guarantee.

#pragma once

#include <stdexcept>
#include <string>
#include <vector>

namespace openarm_mtc {

enum class RunMode {
  PlanOnly,
  OpenOnly,
  PregraspTest,
  ApproachTest,
  CloseTest,
  LiftTest,
  PlaceTest,
  Full,
};

inline RunMode parse_run_mode(const std::string& value) {
  if (value == "plan_only") {
    return RunMode::PlanOnly;
  }
  if (value == "open_only") {
    return RunMode::OpenOnly;
  }
  if (value == "pregrasp_test") {
    return RunMode::PregraspTest;
  }
  if (value == "approach_test") {
    return RunMode::ApproachTest;
  }
  if (value == "close_test") {
    return RunMode::CloseTest;
  }
  if (value == "lift_test") {
    return RunMode::LiftTest;
  }
  if (value == "place_test") {
    return RunMode::PlaceTest;
  }
  if (value == "full") {
    return RunMode::Full;
  }
  throw std::invalid_argument(
      "task.run_mode must be one of plan_only, open_only, pregrasp_test, "
      "approach_test, close_test, lift_test, place_test, full; got '" +
      value + "'");
}

inline std::string run_mode_name(const RunMode mode) {
  switch (mode) {
    case RunMode::PlanOnly:
      return "plan_only";
    case RunMode::OpenOnly:
      return "open_only";
    case RunMode::PregraspTest:
      return "pregrasp_test";
    case RunMode::ApproachTest:
      return "approach_test";
    case RunMode::CloseTest:
      return "close_test";
    case RunMode::LiftTest:
      return "lift_test";
    case RunMode::PlaceTest:
      return "place_test";
    case RunMode::Full:
      return "full";
  }
  return "unknown";
}

inline bool mode_is_full(const RunMode mode) { return mode == RunMode::Full; }

// §12 execution latch: plan_only never executes; every stage mode needs
// execute && allow_execution; full additionally requires allow_full_execution.
inline bool can_execute(const RunMode mode, const bool execute,
                        const bool allow_execution,
                        const bool allow_full_execution) {
  if (mode == RunMode::PlanOnly) {
    return false;
  }
  if (!execute || !allow_execution) {
    return false;
  }
  return !mode_is_full(mode) || allow_full_execution;
}

// How far a run mode's planned pipeline reaches. plan_only plans the complete
// pipeline (same reach as full) but never executes.
enum class PipelineExtent {
  OpenOnly,
  PreGrasp,
  Approach,
  Close,
  Lift,
  Place,
  Full,
};

inline PipelineExtent pipeline_extent_for_mode(const RunMode mode) {
  switch (mode) {
    case RunMode::PlanOnly:
      return PipelineExtent::Full;
    case RunMode::OpenOnly:
      return PipelineExtent::OpenOnly;
    case RunMode::PregraspTest:
      return PipelineExtent::PreGrasp;
    case RunMode::ApproachTest:
      return PipelineExtent::Approach;
    case RunMode::CloseTest:
      return PipelineExtent::Close;
    case RunMode::LiftTest:
      return PipelineExtent::Lift;
    case RunMode::PlaceTest:
      return PipelineExtent::Place;
    case RunMode::Full:
      return PipelineExtent::Full;
  }
  return PipelineExtent::OpenOnly;
}

inline std::string pipeline_extent_name(const PipelineExtent extent) {
  switch (extent) {
    case PipelineExtent::OpenOnly:
      return "open_only";
    case PipelineExtent::PreGrasp:
      return "pregrasp_test";
    case PipelineExtent::Approach:
      return "approach_test";
    case PipelineExtent::Close:
      return "close_test";
    case PipelineExtent::Lift:
      return "lift_test";
    case PipelineExtent::Place:
      return "place_test";
    case PipelineExtent::Full:
      return "full";
  }
  return "unknown";
}

// Predicates shared by the MTC builder and the abstract stage plan so the
// constructed pipeline and the tested plan cannot drift apart.
inline bool extent_uses_pick(const PipelineExtent extent) {
  return extent == PipelineExtent::Approach || extent == PipelineExtent::Close ||
         extent == PipelineExtent::Lift || extent == PipelineExtent::Place ||
         extent == PipelineExtent::Full;
}

inline bool extent_uses_close(const PipelineExtent extent) {
  return extent == PipelineExtent::Close || extent == PipelineExtent::Lift ||
         extent == PipelineExtent::Place || extent == PipelineExtent::Full;
}

inline bool extent_uses_attach_lift(const PipelineExtent extent) {
  return extent == PipelineExtent::Lift || extent == PipelineExtent::Place ||
         extent == PipelineExtent::Full;
}

// place_test lands on the dynamic box-top fraction candidates; lift_test
// reuses the same landing machinery but lands back on the measured grasp XY
// (extent_is_lift_test) so the object does not move.
inline bool extent_uses_place(const PipelineExtent extent) {
  return extent == PipelineExtent::Lift || extent == PipelineExtent::Place ||
         extent == PipelineExtent::Full;
}

inline bool extent_is_lift_test(const PipelineExtent extent) {
  return extent == PipelineExtent::Lift;
}

inline bool extent_uses_return(const PipelineExtent extent) {
  return extent == PipelineExtent::Full;
}

// Abstract ordered stage plan matching the MTC construction in
// mtc_pregrasp.cpp. Used by unit tests to pin the §12 pipeline of every mode.
enum class StageKind {
  CurrentState,
  OpenGripper,
  PreGrasp,
  ConnectPick,
  Approach,
  GraspIK,
  AllowHandObject,
  Close,
  AllowObjectTable,
  Attach,
  Lift,
  ForbidObjectTable,
  AllowPlaceSupport,
  ConnectPlace,
  PlaceLower,
  PlaceIK,
  PlaceOpen,
  PlaceDetach,
  ForbidHandObject,
  ForbidPlaceSupport,
  PlaceRetreat,
  PostReleaseClose,
  ReturnInitial,
};

inline std::string stage_kind_name(const StageKind kind) {
  switch (kind) {
    case StageKind::CurrentState:
      return "CurrentState";
    case StageKind::OpenGripper:
      return "OpenGripper";
    case StageKind::PreGrasp:
      return "PreGrasp";
    case StageKind::ConnectPick:
      return "Connect";
    case StageKind::Approach:
      return "Approach";
    case StageKind::GraspIK:
      return "GraspIK";
    case StageKind::AllowHandObject:
      return "AllowHandObject";
    case StageKind::Close:
      return "Close";
    case StageKind::AllowObjectTable:
      return "AllowObjectTable";
    case StageKind::Attach:
      return "Attach";
    case StageKind::Lift:
      return "Lift";
    case StageKind::ForbidObjectTable:
      return "ForbidObjectTable";
    case StageKind::AllowPlaceSupport:
      return "AllowPlaceSupport";
    case StageKind::ConnectPlace:
      return "ConnectPlace";
    case StageKind::PlaceLower:
      return "PlaceLower";
    case StageKind::PlaceIK:
      return "PlaceIK";
    case StageKind::PlaceOpen:
      return "PlaceOpen";
    case StageKind::PlaceDetach:
      return "PlaceDetach";
    case StageKind::ForbidHandObject:
      return "ForbidHandObject";
    case StageKind::ForbidPlaceSupport:
      return "ForbidPlaceSupport";
    case StageKind::PlaceRetreat:
      return "PlaceRetreat";
    case StageKind::PostReleaseClose:
      return "PostReleaseClose";
    case StageKind::ReturnInitial:
      return "ReturnInitial";
  }
  return "Unknown";
}

inline std::vector<StageKind> stage_plan_for_mode(const RunMode mode) {
  std::vector<StageKind> plan{StageKind::CurrentState, StageKind::OpenGripper};
  const PipelineExtent extent = pipeline_extent_for_mode(mode);
  if (extent == PipelineExtent::PreGrasp) {
    plan.push_back(StageKind::PreGrasp);
  }
  if (extent_uses_pick(extent)) {
    plan.push_back(StageKind::ConnectPick);
    plan.push_back(StageKind::Approach);
    plan.push_back(StageKind::GraspIK);
  }
  if (extent_uses_close(extent)) {
    plan.push_back(StageKind::AllowHandObject);
    plan.push_back(StageKind::Close);
  }
  if (extent_uses_attach_lift(extent)) {
    plan.push_back(StageKind::AllowObjectTable);
    plan.push_back(StageKind::Attach);
    plan.push_back(StageKind::Lift);
    plan.push_back(StageKind::ForbidObjectTable);
  }
  if (extent_uses_place(extent)) {
    plan.push_back(StageKind::AllowPlaceSupport);
    plan.push_back(StageKind::ConnectPlace);
    plan.push_back(StageKind::PlaceLower);
    plan.push_back(StageKind::PlaceIK);
    plan.push_back(StageKind::PlaceOpen);
    plan.push_back(StageKind::PlaceDetach);
    plan.push_back(StageKind::ForbidHandObject);
    plan.push_back(StageKind::ForbidPlaceSupport);
    plan.push_back(StageKind::PlaceRetreat);
  }
  // lift_test reuses the place stages but lands back on the measured grasp XY
  // (extent_is_lift_test) instead of the dynamic box-top fractions.
  if (extent_uses_return(extent)) {
    plan.push_back(StageKind::PostReleaseClose);
    plan.push_back(StageKind::ReturnInitial);
  }
  return plan;
}

// §14 per-mode terminal log line. Only printed when the mode actually ran its
// stages (will_execute), or for plan_only which cannot execute by definition.
inline std::string test_complete_message(const RunMode mode) {
  switch (mode) {
    case RunMode::PlanOnly:
      return "PLAN ONLY COMPLETE - no trajectory was executed";
    case RunMode::OpenOnly:
      return "OPEN TEST COMPLETE - robot stopped after gripper open";
    case RunMode::PregraspTest:
      return "PREGRASP TEST COMPLETE - robot stopped";
    case RunMode::ApproachTest:
      return "APPROACH TEST COMPLETE - gripper remains open";
    case RunMode::CloseTest:
      return "CLOSE TEST COMPLETE - no attach/lift executed";
    case RunMode::LiftTest:
      return "LIFT TEST COMPLETE - object returned to grasp pose and released";
    case RunMode::PlaceTest:
      return "PLACE TEST COMPLETE - robot stopped after retreat";
    case RunMode::Full:
      return "FULL TEST COMPLETE - robot returned to initial joints";
  }
  return "TEST MODE COMPLETE";
}

}  // namespace openarm_mtc
