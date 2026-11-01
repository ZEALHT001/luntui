#!/usr/bin/env python3
"""Regression tests for Shanhai-style per-side support-force wheel gating."""

from __future__ import annotations

import re
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
HEADER = ROOT / "App" / "Inc" / "vmc_probe.h"
MATH_C = ROOT / "App" / "Src" / "vmc_math.c"
MATH_TEST_C = ROOT / "tools" / "test_vmc_leg_force_inverse.c"
PROBE_C = ROOT / "App" / "Src" / "vmc_probe.c"
RTT_C = ROOT / "App" / "Src" / "vmc_lqr_rtt.c"
WEB_INDEX = ROOT / "tools" / "lqr_tuner" / "web" / "index.html"
WEB_MAIN = ROOT / "tools" / "lqr_tuner" / "web" / "src" / "main.js"


def _macro(name: str) -> float:
    source = HEADER.read_text(encoding="utf-8")
    match = re.search(
        rf"^#define\s+{name}\s+(-?[0-9.]+)[fUu]*", source, re.MULTILINE
    )
    if match is None:
        raise AssertionError(f"macro {name} not found in vmc_probe.h")
    return float(match.group(1))


def _function_body(source: str, name: str) -> str:
    pattern = re.compile(
        rf"^(?:static\s+)?(?:uint8_t|void|double)\s+{re.escape(name)}"
        rf"\s*\([^;{{]*\)\s*\{{",
        re.MULTILINE,
    )
    match = pattern.search(source)
    if match is None:
        raise AssertionError(f"definition of {name} not found")
    start = match.end() - 1
    depth = 0
    for index in range(start, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start:index + 1]
    raise AssertionError(f"unterminated body for {name}")


class DefaultsTest(unittest.TestCase):
    def test_uses_shanhai_support_force_cutoff(self) -> None:
        self.assertEqual(_macro("VMC_AIRBORNE_JOINT_TORQUE_THRESHOLD_DEFAULT"), 40.0)
        self.assertEqual(_macro("VMC_AIRBORNE_FN_RELEASE_RATIO_DEFAULT"), 1.25)
        self.assertEqual(_macro("VMC_WHEEL_MASS_KG"), 0.40)

    def test_side_latch_confirms_across_multiple_control_frames(self) -> None:
        self.assertEqual(_macro("VMC_AIRBORNE_SIDE_CONFIRM_MS_DEFAULT"), 20.0)
        self.assertEqual(_macro("VMC_AIRBORNE_SIDE_RELEASE_MS_DEFAULT"), 100.0)

    def test_old_leg_follow_configuration_is_gone(self) -> None:
        header = HEADER.read_text(encoding="utf-8")
        self.assertNotIn("VMC_LEG_FOLLOW_", header)
        self.assertNotIn("leg_follow_", RTT_C.read_text(encoding="utf-8"))
        self.assertNotIn("leg_follow_", WEB_MAIN.read_text(encoding="utf-8"))
        self.assertNotIn("legFollow", WEB_INDEX.read_text(encoding="utf-8"))


class ForceObserverTest(unittest.TestCase):
    def test_inverse_uses_the_same_forward_jacobian(self) -> None:
        body = _function_body(MATH_C.read_text(encoding="utf-8"),
                              "vmc_leg_virtual_force")
        self.assertIn("vmc_leg_force(1.0, 0.0", body)
        self.assertIn("torque_phi0 = joint_torque[0] + joint_torque[1]", body)

    def test_inverse_round_trips_the_compiled_mapping(self) -> None:
        compiler = shutil.which("cc") or shutil.which("clang") or shutil.which("gcc")
        if compiler is None:
            self.skipTest("no host C compiler available")
        with tempfile.TemporaryDirectory() as temp_dir:
            executable = Path(temp_dir) / "vmc_leg_force_inverse"
            build = subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    f"-I{ROOT / 'App' / 'Inc'}",
                    str(MATH_TEST_C),
                    str(MATH_C),
                    "-lm",
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run(
                [str(executable)], capture_output=True, text=True, check=False
            )
            self.assertEqual(run.returncode, 0, run.stderr)

    def test_fn_formula_matches_shanhai(self) -> None:
        body = _function_body(PROBE_C.read_text(encoding="utf-8"),
                              "vmc_side_support_force")
        for term in (
            "force * cos(theta)",
            "torque_phi0 * sin(theta) / leg->length",
            "2.0 * leg->d_length * d_theta * sin(theta)",
            "leg->length * leg->dd_theta * sin(theta)",
            "leg->length * d_theta * d_theta * cos(theta)",
            "VMC_WHEEL_MASS_KG * ddz_w",
            "force = -force",
        ):
            with self.subTest(term=term):
                self.assertIn(term, body)

    def test_side_cutoff_does_not_require_wheel_speed(self) -> None:
        body = _function_body(PROBE_C.read_text(encoding="utf-8"),
                              "vmc_update_airborne_state")
        self.assertIn("side_fn[side] < force_threshold", body)
        self.assertNotIn("side_wheel_speed", body)
        self.assertNotIn("wheels_fast", body)
        self.assertNotIn("attitude_quiet", body)

    def test_only_the_unloaded_wheel_is_gated(self) -> None:
        body = _function_body(PROBE_C.read_text(encoding="utf-8"),
                              "vmc_wheel_control_update")
        self.assertIn(
            "airborne_side_active[VMC_AIRBORNE_SIDE_LEFT] == 0U", body
        )
        self.assertIn(
            "airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT] == 0U", body
        )

    def test_landing_restores_wheel_without_release_delay(self) -> None:
        body = _function_body(PROBE_C.read_text(encoding="utf-8"),
                              "vmc_update_airborne_state")
        self.assertIn("if (side_fn[side] >= release_threshold)", body)
        self.assertNotIn("side_release_ms", body)
        self.assertNotIn(
            "airborne_side_release_candidate_ms[side] +=", body
        )
        self.assertNotIn(
            "airborneSideReleaseMs", WEB_MAIN.read_text(encoding="utf-8")
        )
        self.assertNotIn(
            "airborneSideReleaseMs", WEB_INDEX.read_text(encoding="utf-8")
        )

    def test_fast_telemetry_exposes_left_and_right_fn(self) -> None:
        source = RTT_C.read_text(encoding="utf-8")
        self.assertIn(
            "airborne_side_torque[VMC_AIRBORNE_SIDE_LEFT]", source
        )
        self.assertIn(
            "airborne_side_torque[VMC_AIRBORNE_SIDE_RIGHT]", source
        )

    def test_unloaded_tp_keeps_only_angle_terms(self) -> None:
        control = _function_body(PROBE_C.read_text(encoding="utf-8"),
                                 "vmc_control_update")
        update = _function_body(PROBE_C.read_text(encoding="utf-8"),
                                "update_leg_length_control")
        self.assertIn("-(vmc_balance.left_tp_theta_term +", control)
        self.assertIn("vmc_balance.left_tp_dtheta_term);", control)
        self.assertIn("airborne_posture != 0U", update)
        self.assertIn("clamp_double(airborne_posture_tp", update)

    def test_leg_sync_is_inert_when_either_side_is_unloaded(self) -> None:
        body = _function_body(PROBE_C.read_text(encoding="utf-8"),
                              "vmc_balance_update")
        self.assertIn(
            "airborne_side_active[VMC_AIRBORNE_SIDE_LEFT] == 0U", body
        )
        self.assertIn(
            "airborne_side_active[VMC_AIRBORNE_SIDE_RIGHT] == 0U", body
        )

    def test_leg_target_is_not_rewritten_when_unloaded(self) -> None:
        body = _function_body(PROBE_C.read_text(encoding="utf-8"),
                              "vmc_control_update")
        self.assertNotIn("leg_follow", body)
        self.assertIn(
            "ramped_target_length + roll_comp_offset",
            body,
            "the side target must still come from the normal ramped command",
        )
        self.assertNotIn("left_target_length += offset", body)

    def test_retired_telemetry_slots_remain_six_zeros(self) -> None:
        source = RTT_C.read_text(encoding="utf-8")
        self.assertIn("for (i = 0U; i < 6U; i++)", source)
        self.assertIn("retired leg-follow slots", source)


if __name__ == "__main__":
    unittest.main()
