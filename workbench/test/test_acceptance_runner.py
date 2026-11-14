"""Acceptance CLI validation must reject before starting any process."""
from pathlib import Path
from unittest.mock import patch

import pytest
import yaml

from verification.accept_workbench_fixed_poses import main


@pytest.mark.parametrize('args', [
    ['--domain', '0'], ['--domain', '101'], ['--velocity', 'nan'],
    ['--velocity', '1'], ['--seeds', '1', '1'],
])
def test_bad_options_never_start_process(tmp_path, args):
    with patch('verification.accept_workbench_fixed_poses.ProcessManager') as manager:
        with pytest.raises(SystemExit) as error:
            main(['--poses', str(tmp_path / 'missing.yaml'),
                  '--output', str(tmp_path / 'result'), *args])
    assert error.value.code == 2
    manager.assert_not_called()
    assert not (tmp_path / 'result').exists()


def test_hardware_candidate_never_starts_process(tmp_path):
    source = Path('jaka_competition_kit/config/track1_simulation_example.yaml')
    data = yaml.safe_load(source.read_text())
    data['backend'] = 'hardware'
    path = tmp_path / 'hardware.yaml'
    path.write_text(yaml.safe_dump(data))
    with patch('verification.accept_workbench_fixed_poses.ProcessManager') as manager:
        with pytest.raises(SystemExit) as error:
            main(['--poses', str(path), '--output', str(tmp_path / 'result')])
    assert error.value.code == 2
    manager.assert_not_called()
    assert not (tmp_path / 'result').exists()


def test_previous_evidence_directory_is_not_reused(tmp_path):
    out = tmp_path / 'existing'
    out.mkdir()
    (out / 'score_1.json').write_text('preserve previous evidence')
    with patch('verification.accept_workbench_fixed_poses.ProcessManager') as manager:
        with pytest.raises(FileExistsError):
            main(['--poses', 'jaka_competition_kit/config/track1_simulation_example.yaml',
                  '--output', str(out)])
    manager.assert_not_called()
    assert (out / 'score_1.json').read_text() == 'preserve previous evidence'
