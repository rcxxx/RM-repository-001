TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt

SOURCES += main.cpp \
    serialport.cpp \
    rm_videocapture.cpp \
    rm_armorfitted.cpp \
    rm_bigchrysanthemum.cpp

INCLUDEPATH += /usr/local/include \
                /usr/local/include/opencv
                /usr/local/include/opencv2

LIBS += /usr/local/lib/libopencv_calib3d.so.3.4 \
        /usr/local/lib/libopencv_core.so.3.4 \
        /usr/local/lib/libopencv_dnn.so.3.4 \
        /usr/local/lib/libopencv_features2d.so.3.4 \
        /usr/local/lib/libopencv_flann.so.3.4 \
        /usr/local/lib/libopencv_highgui.so.3.4 \
        /usr/local/lib/libopencv_imgproc.so.3.4 \
        /usr/local/lib/libopencv_ml.so.3.4 \
        /usr/local/lib/libopencv_objdetect.so.3.4 \
        /usr/local/lib/libopencv_photo.so.3.4 \
        /usr/local/lib/libopencv_shape.so.3.4 \
        /usr/local/lib/libopencv_stitching.so.3.4 \
        /usr/local/lib/libopencv_superres.so.3.4 \
        /usr/local/lib/libopencv_video.so.3.4 \
        /usr/local/lib/libopencv_videoio.so.3.4 \
        /usr/local/lib/libopencv_videostab.so.3.4 \
        /lib/libMVSDK.so

/usr/local/lib/libopencv_imgcodecs.so.3.4

HEADERS += \
    configure.h \
    serialport.h \
    rm_videocapture.h \
    rm_armorfitted.h \
    rm_bigchrysanthemum.h \
    rm_kalmanfitted.h
