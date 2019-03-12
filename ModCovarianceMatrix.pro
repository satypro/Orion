QT += core \
    gui \
    opengl \
    network \
    xml \
    widgets

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = tensorvis
TEMPLATE = app

HEADERS += \
    src/Classification/Processing.h \
    src/Classification/pcl_common.h \
    src/Classification/FileRead.h \
    src/Classification/eig3.h \
    src/Classification/Display.h \
    src/Classification/data_type.h \
    src/Classification/cutil_math.h \
    src/Classification/Color.h \
    src/main/TreeWidget.h \
    src/main/Strucparam.h \
    src/main/ParameterWidget.h \
    src/main/mainwindow.h \
    src/main/glwidget.h \
    src/main/FastTrackball.h \
    src/main/doubleSlider.h \
    src/Classification/tensor3dVoting.h \
    src/Classification/OctreeSerial.h \
    src/Classification/octree_pointcloud.h \
    src/Classification/tensor2d.h \
    src/Classification/covariancematrix.h \
    src/Classification/modcovariancematrix.h \
    src/Classification/diffusednormalvoting.h \
    src/Classification/covariancematrix2d.h \
    src/Classification/hessian.h \
    src/Classification/boundarytensor.h \
    src/Classification/graph.h \
    src/Classification/utils.h


SOURCES += \
    src/Classification/Processing.cpp \
    src/Classification/FileRead.cpp \
    src/Classification/Display.cpp \
    src/main/XmdvTool.cpp \
    src/main/TreeWidget.cpp \
    src/main/Strucparam.cpp \
    src/main/ParameterWidget.cpp \
    src/main/mainwindow.cpp \
    src/main/glwidget.cpp \
    src/main/FastTrackball.cpp \
    src/main/doubleSlider.cpp \
    src/Classification/tensor3dVoting.cpp \
    src/Classification/OctreeSerial.cpp \
    src/Classification/Color.cpp \
    src/Classification/tensor2d.cpp \
    src/Classification/covariancematrix.cpp \
    src/Classification/modcovariancematrix.cpp \
    src/Classification/diffusednormalvoting.cpp \
    src/Classification/covariancematrix2d.cpp \
    src/Classification/hessian.cpp \
    src/Classification/boundarytensor.cpp \
    src/Classification/graph.cpp \
    src/Classification/utils.cpp

INCLUDEPATH += src

INCLUDEPATH += /usr/local/include /usr/local/include/pcl-1.8 /usr/include/eigen3 /usr/include/vtk-5.10 /usr/include/jsoncpp

QMAKE_LIBDIR += /usr/local/lib /usr/lib /usr/local/lib/libteem /usr/local/include
LIBS += -lboost_system -lpcl_common -lpcl_io_ply -lpcl_io -lpcl_kdtree -lpcl_keypoints -lpcl_octree -lpcl_filters -lpcl_visualization
LIBS += -lpcl_search -lpcl_filters -lpcl_segmentation -lpcl_features -lpcl_search -lGL -lGLU
LIBS += -L"/usr/include" -lCGAL -lgmp















