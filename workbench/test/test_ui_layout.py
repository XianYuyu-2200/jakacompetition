"""Native Qt layout regression; no ROS, robot, Gazebo or user data is touched."""
import base64
from contextlib import contextmanager
import os
from pathlib import Path
from tempfile import TemporaryDirectory
from unittest.mock import patch

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

import pytest
from PyQt5.QtCore import QByteArray, QBuffer, QIODevice, QPoint, Qt
from PyQt5.QtGui import QColor, QImage
from PyQt5.QtWidgets import QApplication, QPushButton

from workbench.app import WorkbenchWindow
from workbench.theme import COLORS, stylesheet

ROOT = Path(__file__).resolve().parents[2]
APP = QApplication.instance() or QApplication([])


@contextmanager
def window(width=1366, height=768, scale=1):
    with TemporaryDirectory() as directory, patch.dict(os.environ, WORKBENCH_UI_SCALE=str(scale)):
        widget = WorkbenchWindow(ROOT, Path(directory), demo=True)
        widget.resize(width, height)
        widget.show()
        APP.processEvents()
        try:
            yield widget
        finally:
            widget.shutdown()
            widget.close()
            APP.processEvents()


def fully_inside(child, ancestor):
    origin = child.mapTo(ancestor, QPoint(0, 0))
    return ancestor.rect().contains(origin) and ancestor.rect().contains(
        origin + QPoint(child.width() - 1, child.height() - 1))


@pytest.mark.parametrize('width,height,scale', [
    (960, 620, 1), (1024, 768, 1), (1366, 768, 1), (1920, 1080, 1),
    (1366, 768, 1.75), (1920, 1080, 1.75),
])
def test_vm_layout_keeps_images_stop_and_controls_accessible(width, height, scale):
    with window(width, height, scale) as w:
        assert w.width() == width and w.height() == height
        assert not w.log_frame.isVisible()
        for index in range(3):
            w.inspector.setCurrentIndex(index)
            APP.processEvents()
            scroll = w.inspector.widget(index)
            body = scroll.widget()
            # Offscreen below the fold is intentional; horizontal clipping is not.
            assert body.width() <= scroll.viewport().width()
            for button in body.findChildren(QPushButton):
                assert fully_inside(button, body), button.text()
                assert button.height() >= round(44 * scale)
            for image in (w.gazebo_placeholder, w.camera_placeholder):
                assert image.isVisible()
                assert fully_inside(image, w.gazebo_view)
            assert fully_inside(w.stop_round_button, w.pages)
            assert w.stop_round_button.isVisible()
        for button in w.nav_buttons:
            available = button.width() - round(24 * scale)
            assert button.fontMetrics().horizontalAdvance(button.text()) <= available
        assert not w.manager.active_names()


def test_joint_fields_do_not_overlap_and_record_button_is_scroll_reachable():
    with window(1024, 768) as w:
        w.inspector.setCurrentIndex(1)
        APP.processEvents()
        fields = [w.pose_station, w.pose_phase] + w.joint_spins
        parent = w.joint_spins[0].parentWidget()
        for i, field in enumerate(fields):
            assert field.accessibleName()
            assert fully_inside(field, parent)
            for other in fields[i + 1:]:
                assert not field.geometry().intersects(other.geometry())
        scroll = w.inspector.currentWidget()
        assert scroll.verticalScrollBar().maximum() > 0
        record = next(b for b in scroll.findChildren(QPushButton) if b.text() == '记录此姿态')
        scroll.ensureWidgetVisible(record)
        APP.processEvents()
        assert fully_inside(record, scroll.viewport())
        w.start_environment()
        w.generate()
        w.run_reference()
        assert w.stop_round_button.isEnabled()
        w.stop_round_button.click()
        assert w.run_state == 'finished'


def test_log_drawer_toggle_and_navigation_keep_existing_actions():
    with window() as w:
        w.log_toggle.click()
        APP.processEvents()
        assert w.log_frame.isVisible()
        assert w.log_toggle.text() == '收起日志'
        assert fully_inside(w.log_frame, w.centralWidget())
        assert fully_inside(w.camera_placeholder, w.gazebo_view)
        w.log_toggle.click()
        for index, title in enumerate(('练习控制台', '我的程序', '运行记录', '入门指南')):
            w.nav_buttons[index].click()
            assert w.pages.currentIndex() == index
            assert w.page_title.text() == title
            assert sum(b.isChecked() for b in w.nav_buttons) == 1


def test_camera_retains_real_image_across_inspector_tabs_and_resizing():
    with window() as w:
        # Synthetic flat image verifies display plumbing, not camera/simulation acceptance.
        image = QImage(640, 480, QImage.Format_RGB32)
        image.fill(QColor('#537895'))
        data = QByteArray()
        buffer = QBuffer(data)
        buffer.open(QIODevice.WriteOnly)
        assert image.save(buffer, 'JPG')
        payload = {'jpeg': base64.b64encode(bytes(data)).decode()}
        for width in (1366, 1920, 1024):
            w.resize(width, 768)
            APP.processEvents()
            assert w.view_split.orientation() == (Qt.Horizontal if width >= 1700 else Qt.Vertical)
            for index in range(3):
                w.inspector.setCurrentIndex(index)
                APP.processEvents()
                w._show_camera_image(payload)
                pixmap = w.camera_placeholder.pixmap()
                assert pixmap is not None and not pixmap.isNull()
                assert pixmap.width() <= w.camera_placeholder.contentsRect().width()
                assert pixmap.height() <= w.camera_placeholder.contentsRect().height()
                assert abs(pixmap.width() / pixmap.height() - 4 / 3) < .03


def test_missing_dependencies_reveal_environment_tab():
    with window() as w:
        w._start_after_check = True
        w._checked({'checks': [{'name': 'ROS', 'level': 'error', 'detail': 'missing'}],
                    'conflicts': []})
        assert w.inspector.currentIndex() == 2
        assert w.check_table.rowCount() == 1
        assert '环境设置' in w.notice.text()


def test_theme_tokens_focus_and_readability():
    css = stylesheet(1)
    assert '$' not in css and '#176BDB_hover' not in css
    assert 'QPushButton:focus' in css
    assert '26px' in stylesheet(1.75)  # larger body text is preserved

    def luminance(hex_color):
        rgb = [int(hex_color[i:i + 2], 16) / 255 for i in (1, 3, 5)]
        linear = [c / 12.92 if c <= .04045 else ((c + .055) / 1.055) ** 2.4 for c in rgb]
        return sum(c * weight for c, weight in zip(linear, (.2126, .7152, .0722)))

    for foreground, background in [('ink', 'surface'), ('muted', 'canvas'),
                                    ('surface', 'accent'), ('surface', 'accent_hover'),
                                    ('surface', 'nav'), ('danger', 'surface')]:
        a, b = sorted((luminance(COLORS[foreground]), luminance(COLORS[background])))
        assert (b + .05) / (a + .05) >= 4.5
