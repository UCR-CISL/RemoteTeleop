from pathlib import Path

import pytest

from scripts.deployment.stage_remote_teleop_batch import BatchCodeStager


def test_stage_uses_isolated_code_and_dependencies_without_deleting_or_launching():
    stager = BatchCodeStager(Path('cfg/alien3_lambda_proxy_batch.yaml'), Path('cfg/alien3_lambda.yaml'), Path('.'))
    commands = list(stager.commands())
    transfers = [command for command in commands if command[0] == 'rsync']
    assert len(transfers) == 2
    assert all('RemoteTeleop-proxy-batch/' in command[-1] for command in transfers)
    text = '\n'.join(' '.join(command) for command in commands)
    assert '--delete' not in text
    assert '--target' in text and '.batch-site' in text
    assert 'setsid' not in text and 'nohup' not in text
    assert 'batch-python' in text


def test_stage_refuses_existing_repository_paths():
    with pytest.raises(ValueError, match='separate repository'):
        BatchCodeStager(Path('cfg/alien3_lambda.yaml'), Path('cfg/alien3_lambda.yaml'), Path('.'))
