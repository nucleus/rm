/*
 * Header file for CG2EX2 QT main window.
 */

#ifndef EX2_MAIN_WINDOW_HPP
#define EX2_MAIN_WINDOW_HPP

#include <QMainWindow>

class MainWindow : public QMainWindow {
	Q_OBJECT
public:
	MainWindow(QMainWindow * parent = 0);
	~MainWindow();
private:
	void setupUi();

	class GLWidget * glWidget;
	class QDockWidget * uiDock;

	class QLabel* uiTreeDepthLabel;
	class QSpinBox* uiTreeDepth;

	class QLabel* uiLeafSizeLabel;
	class QSpinBox* uiLeafSize;
	
	class QLabel* uiGridDimensionsLabel;
	class QSpinBox* uiGridDimensions[3];
	
	class QLabel* uiRadiusLabel;
	class QDoubleSpinBox* uiRadius;
	
	class QLabel* uiStepsLabel;
	class QSpinBox* uiSteps;
	
	class QPushButton* uiLoadFileButton;

	class QCheckBox* uiRenderPoints;
	class QCheckBox* uiRenderGrid;
	class QCheckBox* uiRenderWLS;
	class QCheckBox* uiRenderRM;
	
	class QRadioButton* uiDeviceSelector[2];
	
	class QPushButton* uiWLSButton;
	
	class QPushButton* uiRaymarchButton;

public:
	void keyPressEvent(QKeyEvent *event);
	
private slots:
};

#endif
