"""Desktop identity must not put UTF-8 bytes in the legacy X11 WM_CLASS."""
from pathlib import Path
from unittest.mock import MagicMock, patch


def test_entrypoint_separates_ascii_identity_from_chinese_display_name(tmp_path):
    from workbench.__main__ import main

    app = MagicMock()
    app.exec_.return_value = 0
    root = Path(__file__).resolve().parents[2]
    with patch('PyQt5.QtWidgets.QApplication', return_value=app) as qt, \
         patch('workbench.app.WorkbenchWindow') as window, \
         patch.dict('os.environ', QT_QPA_PLATFORM='offscreen'):
        assert main(['--demo', '--root', str(root), '--data-dir', str(tmp_path)]) == 0
    qt.assert_called_once_with(['jaka-competition-workbench'])
    app.setApplicationName.assert_called_once_with('jaka-competition-workbench')
    app.setApplicationDisplayName.assert_called_once_with('基础组学习工作台')
    app.setDesktopFileName.assert_called_once_with('jaka-competition-workbench')
    window.return_value.show.assert_called_once()
