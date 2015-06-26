TEMPLATE = app
CONFIG += console
CONFIG -= qt

SOURCES += \
    segImageExtractFeatures.cpp \
    vision/svlRegionFeatures.cpp \
    base/svlCodeProfiler.cpp \
    base/svlConfigManager.cpp \
    base/svlLogger.cpp \
    base/svlOptions.cpp \
    base/svlThreadPool.cpp \
    base/svlFileUtils.cpp \
    vision/svlSegImage.cpp \
    vision/svlConvolution.cpp \
    vision/svlVisionUtils.cpp \
    vision/svlOpenCVUtils.cpp \
    ml/svlDisjointSets.cpp \
    xmlParser/xmlParser.cpp \
    base/svlStrUtils.cpp

HEADERS += \
    svlBase.h \
    svlVision.h \
    vision/svlRegionFeatures.h \
    base/svlCodeProfiler.h \
    base/svlConfigManager.h \
    base/svlLogger.h \
    base/svlOptions.h \
    base/svlThreadPool.h \
    base/svlFileUtils.h \
    vision/svlSegImage.h \
    vision/svlConvolution.h \
    vision/svlVisionUtils.h \
    vision/svlOpenCVUtils.h \
    svlML.h \
    ml/svlDisjointSets.h \
    xmlParser/xmlParser.h \
    base/svlStrUtils.h

INCLUDEPATH += eigen-2.0.16/ \
               ../../external/opencv/include/opencv/ \

QMAKE_LIBS = -lopencv_core \
            -lopencv_imgproc \
            -lopencv_highgui \
            -lpthread \

