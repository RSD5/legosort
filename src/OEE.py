# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'OEE.ui'
#
# Created: Thu Feb 28 08:26:27 2013
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(337, 281)
        Form.setAutoFillBackground(False)
        self.layoutWidget = QtGui.QWidget(Form)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 20, 321, 251))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.verticalLayout = QtGui.QVBoxLayout(self.layoutWidget)
        self.verticalLayout.setMargin(0)
        self.verticalLayout.setObjectName(_fromUtf8("verticalLayout"))
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.oee_label = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(48)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.oee_label.setFont(font)
        self.oee_label.setObjectName(_fromUtf8("oee_label"))
        self.horizontalLayout_4.addWidget(self.oee_label)
        self.oee_output = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(48)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.oee_output.setFont(font)
        self.oee_output.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.oee_output.setOpenExternalLinks(False)
        self.oee_output.setObjectName(_fromUtf8("oee_output"))
        self.horizontalLayout_4.addWidget(self.oee_output)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.availability_label = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.availability_label.setFont(font)
        self.availability_label.setObjectName(_fromUtf8("availability_label"))
        self.horizontalLayout_3.addWidget(self.availability_label)
        self.availability_output = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.availability_output.setFont(font)
        self.availability_output.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.availability_output.setObjectName(_fromUtf8("availability_output"))
        self.horizontalLayout_3.addWidget(self.availability_output)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.performance_label = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.performance_label.setFont(font)
        self.performance_label.setObjectName(_fromUtf8("performance_label"))
        self.horizontalLayout_2.addWidget(self.performance_label)
        self.performance_output = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.performance_output.setFont(font)
        self.performance_output.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.performance_output.setObjectName(_fromUtf8("performance_output"))
        self.horizontalLayout_2.addWidget(self.performance_output)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.quality_label = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.quality_label.setFont(font)
        self.quality_label.setObjectName(_fromUtf8("quality_label"))
        self.horizontalLayout.addWidget(self.quality_label)
        self.quality_output = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.quality_output.setFont(font)
        self.quality_output.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.quality_output.setObjectName(_fromUtf8("quality_output"))
        self.horizontalLayout.addWidget(self.quality_output)
        self.verticalLayout.addLayout(self.horizontalLayout)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtGui.QApplication.translate("Form", "Form", None, QtGui.QApplication.UnicodeUTF8))
        self.oee_label.setText(QtGui.QApplication.translate("Form", "OEE", None, QtGui.QApplication.UnicodeUTF8))
        self.oee_output.setText(QtGui.QApplication.translate("Form", "755", None, QtGui.QApplication.UnicodeUTF8))
        self.availability_label.setText(QtGui.QApplication.translate("Form", "Availability", None, QtGui.QApplication.UnicodeUTF8))
        self.availability_output.setText(QtGui.QApplication.translate("Form", "811", None, QtGui.QApplication.UnicodeUTF8))
        self.performance_label.setText(QtGui.QApplication.translate("Form", "Peformance", None, QtGui.QApplication.UnicodeUTF8))
        self.performance_output.setText(QtGui.QApplication.translate("Form", "912", None, QtGui.QApplication.UnicodeUTF8))
        self.quality_label.setText(QtGui.QApplication.translate("Form", "Quality", None, QtGui.QApplication.UnicodeUTF8))
        self.quality_output.setText(QtGui.QApplication.translate("Form", "990", None, QtGui.QApplication.UnicodeUTF8))

