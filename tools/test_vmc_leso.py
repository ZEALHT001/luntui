import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))


class LesoObserverTest(unittest.TestCase):
    def test_observer_converges_to_a_constant_disturbance(self) -> None:
        """The observer is what removes a slope's steady-state error, so the
        host test drives a stabilised plant with a constant input disturbance
        and checks that the estimate reaches it."""
        compiler = (
            shutil.which("cc") or shutil.which("clang") or shutil.which("gcc")
        )
        if compiler is None:
            self.skipTest("no host C compiler available")
        with tempfile.TemporaryDirectory() as temporary:
            executable = Path(temporary) / "test_vmc_leso"
            build = subprocess.run(
                [
                    compiler,
                    "-std=c11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    f"-I{ROOT / 'App' / 'Inc'}",
                    str(ROOT / "tools" / "test_vmc_leso.c"),
                    str(ROOT / "App" / "Src" / "vmc_leso.c"),
                    str(ROOT / "App" / "Src" / "vmc_leso_model.c"),
                    "-lm",
                    "-o",
                    str(executable),
                ],
                capture_output=True,
                text=True,
            )
            self.assertEqual(build.returncode, 0, f"build failed:\n{build.stderr}")
            run = subprocess.run([str(executable)], capture_output=True, text=True)
            self.assertEqual(
                run.returncode, 0, f"test failed:\n{run.stdout}\n{run.stderr}"
            )

    def test_model_matches_the_generator(self) -> None:
        import numpy as np

        from derive_vmc_lqr_abk import PRESETS, emit_leso_model

        lengths = np.linspace(0.180, 0.385, num=9)
        expected = emit_leso_model(PRESETS["abk"], lengths, 0.002222)
        actual = (ROOT / "App" / "Src" / "vmc_leso_model.c").read_text(
            encoding="utf-8"
        )
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
