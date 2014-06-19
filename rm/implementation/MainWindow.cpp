/*
 * Implementation file for CG2EX2 QT main window.
 */

#include <rm/interface/MainWindow.hpp>
#include <rm/interface/GLWidget.hpp>

#include <QCheckBox>
#include <QRadioButton>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QDockWidget>
#include <QGridLayout>
#include <QSignalMapper>
#include <QPushButton>
#include <QKeyEvent>

MainWindow::MainWindow(QMainWindow * parent) : QMainWindow(parent)
{
	setupUi();
	
	connect(uiLeafSize, SIGNAL(valueChanged(int)), glWidget, SLOT(setTreeLeafSize(int)));
	connect(uiTreeDepth, SIGNAL(valueChanged(int)), glWidget, SLOT(setTreeMaxDepth(int)));

	connect(uiGridDimensions[0], SIGNAL(valueChanged(int)), glWidget, SLOT(setGridDimX(int)));
	connect(uiGridDimensions[1], SIGNAL(valueChanged(int)), glWidget, SLOT(setGridDimY(int)));
	connect(uiGridDimensions[2], SIGNAL(valueChanged(int)), glWidget, SLOT(setGridDimZ(int)));
	
	connect(uiRadius, SIGNAL(valueChanged(double)), glWidget, SLOT(setRadius(double)));
	connect(uiSteps, SIGNAL(valueChanged(int)), glWidget, SLOT(setRMSteps(int)));
	
	connect(uiLoadFileButton, SIGNAL(clicked()), glWidget, SLOT(loadFile()));
	
	connect(uiRenderPoints, SIGNAL(toggled(bool)), glWidget, SLOT(setRenderPoints(bool)));
	connect(uiRenderGrid, SIGNAL(toggled(bool)), glWidget, SLOT(setRenderGrid(bool)));
	connect(uiRenderWLS, SIGNAL(toggled(bool)), glWidget, SLOT(setRenderWLS(bool)));
	connect(uiRenderRM, SIGNAL(toggled(bool)), glWidget, SLOT(setRenderRM(bool)));
	connect(uiImmediateMode, SIGNAL(toggled(bool)), glWidget, SLOT(setImmediateMode(bool)));

	connect(uiRenderRMPoints, SIGNAL(valueChanged(int)), glWidget, SLOT(setMaxRenderPoints(int)));
	
	connect(uiDeviceSelector[0], SIGNAL(pressed()), glWidget, SLOT(enableCPUDevice()));
	connect(uiDeviceSelector[1], SIGNAL(pressed()), glWidget, SLOT(enableGPUDevice()));
	
	connect(uiWLSButton, SIGNAL(clicked()), glWidget, SLOT(wls()));
	connect(uiRaymarchButton, SIGNAL(clicked()), glWidget, SLOT(raymarch()));

	// default settings
	uiLeafSize->setValue(8);
	uiTreeDepth->setValue(24);
	
	for (int i = 0; i < 3; i++) { uiGridDimensions[i]->setValue(10); }
	
	uiRadius->setValue(0.5);
	uiRadius->setSingleStep(0.1);
	
	uiSteps->setValue(100);
	uiSteps->setSingleStep(5);
	
	uiRenderRMPoints->setValue(100000);
	uiRenderRMPoints->setSingleStep(10000);
	
	uiDeviceSelector[0]->setChecked(true);
	
    uiRenderPoints->setChecked(true);
}

MainWindow::~MainWindow() {

}

void MainWindow::setupUi()
{
	glWidget = new GLWidget();

	setCentralWidget(glWidget);
	
	resize(1024, 768);

	setFocusPolicy(Qt::NoFocus);
	setContextMenuPolicy(Qt::ActionsContextMenu);
	setAutoFillBackground(true);

	QGridLayout * layout = new QGridLayout();

	unsigned row = 0;
	
	uiLeafSizeLabel = new QLabel("Leaf size");
	layout->addWidget(uiLeafSizeLabel, row, 0, 1, 1, Qt::AlignTop);
	uiLeafSize = new QSpinBox();
	uiLeafSize->setMinimum(1);
	uiLeafSize->setMaximum(99999);
    layout->addWidget(uiLeafSize, row++, 1, 1, 1, Qt::AlignTop);

	uiTreeDepthLabel = new QLabel("Depth");
    layout->addWidget(uiTreeDepthLabel, row, 0, 1, 1, Qt::AlignTop);
	uiTreeDepth = new QSpinBox();
	uiTreeDepth->setMinimum(1);
	uiTreeDepth->setMaximum(100);
    layout->addWidget(uiTreeDepth, row++, 1, 1, 1, Qt::AlignTop);

	uiGridDimensionsLabel = new QLabel("Grid Dimensions");
	layout->addWidget(uiGridDimensionsLabel, row++, 0, 1, 1, Qt::AlignTop);
	for (unsigned i = 0; i < 3; i++) {
		uiGridDimensions[i] = new QSpinBox();
		uiGridDimensions[i]->setMinimum(2);
		uiGridDimensions[i]->setMaximum(1000);
		layout->addWidget(uiGridDimensions[i], row, i, 1, 1, Qt::AlignTop);
	}
	row++;
	
	uiRadiusLabel = new QLabel("Radius");
	layout->addWidget(uiRadiusLabel, row, 0, 1, 1, Qt::AlignTop);
	uiRadius = new QDoubleSpinBox();
	layout->addWidget(uiRadius, row++, 1, 1, 1, Qt::AlignTop);
	
	uiStepsLabel = new QLabel("Steps");
	layout->addWidget(uiStepsLabel, row, 0, 1, 1, Qt::AlignTop);
	uiSteps = new QSpinBox();
	uiSteps->setMinimum(10);
	uiSteps->setMaximum(1000000);
	layout->addWidget(uiSteps, row++, 1, 1, 1, Qt::AlignTop);

	uiLoadFileButton = new QPushButton("Load.off");
	layout->addWidget(uiLoadFileButton, row++, 0, 1, 3, Qt::AlignTop);

	uiRenderPoints = new QCheckBox("Render Points");
	layout->addWidget(uiRenderPoints, row++, 0, 1, 3, Qt::AlignTop);
	uiRenderGrid = new QCheckBox("Render Grid");
	layout->addWidget(uiRenderGrid, row++, 0, 1, 3, Qt::AlignTop);
	uiRenderWLS = new QCheckBox("Render WLS Approximation");
	layout->addWidget(uiRenderWLS, row++, 0, 1, 3, Qt::AlignTop);
	uiRenderRM = new QCheckBox("Render Raymarch Results");
	layout->addWidget(uiRenderRM, row++, 0, 1, 3, Qt::AlignTop);
	uiImmediateMode = new QCheckBox("Immediate Mode");
	layout->addWidget(uiImmediateMode, row++, 0, 1, 3, Qt::AlignTop);
    
	uiRenderRMPointsLabel = new QLabel("Render max. points");
	layout->addWidget(uiRenderRMPointsLabel, row, 0, 1, 2, Qt::AlignTop);
	uiRenderRMPoints = new QSpinBox();
	uiRenderRMPoints->setMinimum(10000);
	uiRenderRMPoints->setMaximum(10000000);
	layout->addWidget(uiRenderRMPoints, row++, 2, 1, 1, Qt::AlignTop);
	
	uiDeviceSelector[0] = new QRadioButton("CPU");
	layout->addWidget(uiDeviceSelector[0], row, 0, 1, 1, Qt::AlignTop);
	uiDeviceSelector[1] = new QRadioButton("GPU");
	layout->addWidget(uiDeviceSelector[1], row++, 1, 1, 1, Qt::AlignTop);
	
	uiWLSButton = new QPushButton("Compute WLS");
	layout->addWidget(uiWLSButton, row++, 0, 1, 3, Qt::AlignTop);
	
	uiRaymarchButton = new QPushButton("Raymarch!");
	layout->addWidget(uiRaymarchButton, row++, 0, 1, 3, Qt::AlignTop);

	uiDock = new QDockWidget();
	QWidget * tmpWidget = new QWidget();
	uiDock->setWidget(tmpWidget);
	tmpWidget->setLayout(layout);
	addDockWidget(Qt::RightDockWidgetArea, uiDock);
}

/////////////////////////////////////////////////////////////////////////

void MainWindow::keyPressEvent(QKeyEvent* event) {
	switch (event->key()) {
		case Qt::Key_Escape:
			close();
			break;
		default: break;
	}
}




