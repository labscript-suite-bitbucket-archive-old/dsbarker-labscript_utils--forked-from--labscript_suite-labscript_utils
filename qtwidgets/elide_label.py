import sys
if 'PySide' in sys.modules:
    from PySide.QtCore import *
    from PySide.QtGui import *
else:
    from PyQt4.QtCore import *
    from PyQt4.QtGui import *

from qtutils import *


ELLIPSIS = u'\u2026'


class ElideScrollArea(QScrollArea):
    """A ScrollArea for containing a label that we want to elide. The elision
    is attained by just letting the text we don't want to see be scrolled off
    to the side with the scrollbars hidden."""
    def __init__(self, *args, **kwargs):
        QScrollArea.__init__(self, *args, **kwargs)
        self.horizontalScrollBar().setStyleSheet("QScrollBar {height:0px;}");
        self.verticalScrollBar().setStyleSheet("QScrollBar {width:0px;}");
        self.setFrameStyle(QFrame.NoFrame)
        self.setElideMode(Qt.ElideNone)
        self.setWidgetResizable(True)
        
    def setElideMode(self, elideMode):
        if not isinstance(elideMode, Qt.TextElideMode):
            raise TypeError("Argument must be of type Qt.TextElideMode")
        if elideMode == Qt.ElideMiddle:
            raise NotImplementedError("The hack being used to elidetext does not work for ElideMiddle")

        self._elideMode = elideMode
        
    def minimumSizeHint(self):
        if self.widget is None or self._elideMode == Qt.ElideNone:
            return QScrollArea.minimumSizeHint(self)
        else:
            actual_minimum_sizehint = self.widget().minimumSizeHint()
            return QSize(0, actual_minimum_sizehint.height())

    def sizeHint(self):
        if self.widget is None or self._elideMode == Qt.ElideNone:
            return QScrollArea.sizeHint(self)
        else:
            actual_sizehint = self.widget().sizeHint()
            return QSize(0, actual_sizehint.height())

    def setWidget(self, widget):
        QScrollArea.setWidget(self, widget)
        self.setSizePolicy(QSizePolicy(self.sizePolicy().horizontalPolicy(), widget.sizePolicy().verticalPolicy()))


class ElidedLabelContainer(QWidget):
    """A QWidget to contain a QLabel with a single line of (possibly rich)
    text that we want to elide. The elision is obtained by putting the QLabel
    in a QScrollArea and having the QScrollarea only show the part of the text
    we want to see. An extra label with the elision indication "..." is also
    inserted next to the QScrollArea.
    """
    def __init__(self, label):
        QWidget.__init__(self)
        self.label = label
        self.layout = QHBoxLayout(self)
        self.layout.setSpacing(0)
        self.layout.setContentsMargins(0,0,0,0)
        self.ellipsis_label = QLabel(ELLIPSIS)
        self.scroll_area = ElideScrollArea()
        self.scroll_area.setWidget(self.label)
        self.setElideMode(Qt.ElideNone)
        self.setSizePolicy(label.sizePolicy())
        self.scroll_area.horizontalScrollBar().rangeChanged.connect(self.update_elide_widget)

    def elideMode(self):
        return self._elideMode

    def setElideMode(self, elideMode):
        if not isinstance(elideMode, Qt.TextElideMode):
            raise TypeError("Argument must be of type Qt.TextElideMode")
        if elideMode == Qt.ElideMiddle:
            raise NotImplementedError("The hack being used to elidetext does not work for ElideMiddle")

        self._elideMode = elideMode
        self.scroll_area.setElideMode(self._elideMode)

        if self.layout.count():
            self.layout.removeWidget(self.ellipsis_label)
            self.layout.removeWidget(self.scroll_area)
        if self._elideMode == Qt.ElideLeft:
            self.layout.addWidget(self.ellipsis_label)
            self.layout.addWidget(self.scroll_area)
        elif self._elideMode == Qt.ElideRight:
            self.layout.addWidget(self.scroll_area)
            self.layout.addWidget(self.ellipsis_label)

    def resizeEvent(self, event):
        result = QWidget.resizeEvent(self, event)
        self.update_elide_widget()
        return result

    def update_elide_widget(self):
        label_width = self.label.minimumSizeHint().width()

        width = self.width()
        if label_width > width:
            self.ellipsis_label.show()
        else:
            self.ellipsis_label.hide()

        if self._elideMode == Qt.ElideNone:
            return
        elif self._elideMode == Qt.ElideLeft:
            self.scroll_area.ensureVisible(label_width, 0, 0, 0)
        elif self._elideMode == Qt.ElideRight:
            self.scroll_area.ensureVisible(0, 0, 0, 0)


def elide_label(label, layout, elide_mode):
    """Take an existing label that is in a layout, and wrap it in our widgets
    that elide the text, and insert it back into the layout. This is a hack
    that allows us to elide a QLabel with a single line of (possibly rich)
    text, a task that seems pretty much impossible to do in any kosher way.

    This function is for modifying an existing label already in a layout, but
    if you are programatically creating a label, then you can wrap it in
    ElidedLabelContainer(label) before inserting it into a layout or other
    container widget, which is more flexible than this function which only
    works if the label is in a QBoxLayout"""
    if not (isinstance(layout, QBoxLayout) or isinstance(layout, QSplitter)):
        raise NotImplementedError("Only labels that are in QBoxLayouts or QSplitters supported")
    index = layout.indexOf(label)
    container = ElidedLabelContainer(label)
    layout.insertWidget(index, container)
    container.setElideMode(elide_mode)
        


if __name__ == '__main__':
    # test:
    
    test_text = "The <b>quick</b> brown fox <b>jumped over the lazy dog</b>"
    app = QApplication(sys.argv)
    window = QWidget()
    hlayout = QHBoxLayout(window)
    widget = QWidget()
    layout = QVBoxLayout(widget)
    normal_label = QLabel("Normal label")
    normal_label.setStyleSheet("QLabel { background-color : red; color : blue; }")
    hlayout.addWidget(normal_label)
    hlayout.addWidget(widget)

    elide_left = QLabel("ElideLeft: " + test_text)
    elide_right = QLabel("ElideRight: " + test_text)
    smaller_label = QLabel("Smaller label")
    smaller_label2 = QLabel("Smaller label")
    smaller_label3 = QLabel("Smaller label")
    smaller_label.setStyleSheet("QLabel { background-color : red; color : blue; }")
    smaller_label2.setStyleSheet("QLabel { background-color : red; color : blue; }")
    smaller_label3.setStyleSheet("QLabel { background-color : red; color : blue; }")

    layout.setSpacing(0)

    layout.addWidget(smaller_label)
    layout.addWidget(elide_left)
    layout.addWidget(smaller_label2)
    layout.addWidget(elide_right)
    layout.addWidget(smaller_label3)

    window.show()
    window.resize(20, 20)

    elide_label(elide_left, layout, Qt.ElideLeft)
    elide_label(elide_right, layout, Qt.ElideRight)

    def foo():
        elide_left.setText("ElideLeft: " + test_text + test_text)

    QTimer.singleShot(3000, foo)
    app.exec_()
