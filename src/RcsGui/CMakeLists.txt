
SET(RCS_HIGHGUI_SRCS
HighGuiWindow.cpp
HighGuiPlot.cpp
HighGuiLabel.cpp
HighGuiImage.cpp
HighGuiButton.cpp
HighGuiWidget.cpp
HighGui.cpp
HighGuiSlider.cpp
)

SET(RCS_GUI_SRCS
ControllerWidgetBase.cpp
ForceControlGui.cpp
GuiFactory.cpp
JointSlider.cpp
JointWidget.cpp
LcdSlider.cpp
MatNdWidget.cpp
PPSGui.cpp
PPSWidget.cpp
Rcs_guiFactory.c
SimpleSlider.cpp
Slider1Dof.cpp
SliderCheck1Dof.cpp
TaskWidget.cpp
DblSlider.cpp
${RCS_HIGHGUI_SRCS}
)


IF(WIN32)
  SET_SOURCE_FILES_PROPERTIES( ${RCS_GUI_SRCS} PROPERTIES LANGUAGE CXX )
  
  IF(CREATE_WIN_DLL)
    ADD_DEFINITIONS(-DRCSGUI_EXPORTS)
  ENDIF(CREATE_WIN_DLL)

ENDIF()

SET(RCS_HIGHGUI_HEADERS
HighGuiLabel.h
HighGuiSlider.h
HighGuiButton.h
HighGuiWindow.h
HighGuiPlot.h
HighGuiWidget.h
)

SET(RCS_Qt4_SRC
ControllerWidgetBase.h
ForceControlGui.h
GuiFactory.h
JointSlider.h
JointWidget.h
LcdSlider.h
MatNdWidget.h
SimpleSlider.h
Slider1Dof.h
SliderCheck1Dof.h
TaskWidget.h
PPSGui.h
PPSWidget.h
DblSlider.h
${RCS_HIGHGUI_HEADERS}
)

IF (MSVC_VERSION STREQUAL 1900 OR NOT QWT_MAJOR_VERSION VERSION_LESS 6)
  MESSAGE("-- Compiling RcsGui with Qt5 libraries")
  ADD_LIBRARY(RcsGui ${LIBRARY_LINKING_TYPE} ${RCS_GUI_SRCS} )
  TARGET_LINK_LIBRARIES(RcsGui RcsCore Qt5::Core Qt5::Gui Qt5::Widgets ${QWT_LIBRARIES})
  SET_TARGET_PROPERTIES(RcsGui PROPERTIES AUTOMOC TRUE)
ELSE()
  MESSAGE("-- Compiling RcsGui with Qt4 libraries")
  SET_PROPERTY(SOURCE HighGui.cpp HighGuiWindow.cpp PROPERTY COMPILE_DEFINITIONS WITH_X11_SUPPORT)
  SET(RCS_Qt4_RES )
  QT4_ADD_RESOURCES(RCS_RES_H ${RCS_Qt4_RES})
  QT4_WRAP_UI(RCS_UI_CPP ${RCS_Qt4_UI})
  QT4_WRAP_CPP(RCS_MOC_CPP ${RCS_Qt4_SRC})
  ADD_LIBRARY(RcsGui ${LIBRARY_LINKING_TYPE} ${RCS_GUI_SRCS} ${RCS_MOC_CPP} ${RCS_UI_CPP} ${RCS_RES_H})
  TARGET_LINK_LIBRARIES(RcsGui RcsCore ${QWT_LIBRARIES} Qt4::QtGui)
ENDIF()

TARGET_INCLUDE_DIRECTORIES(RcsGui PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsGui>
    
    # Regular dependencies
    ${QT_QTGUI_INCLUDE_DIR}
    ${QT_QTCORE_INCLUDE_DIR}
    ${QT_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(RcsGui PRIVATE ${QWT_INCLUDE_DIRS})

# Mark for export
RCS_ADD_EXPORT_LIBRARY(RcsGui)

# Installation
# Headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsGui
  FILES_MATCHING PATTERN "*.h"
)
# Install the binaries
INSTALL(TARGETS RcsGui EXPORT RcsExport DESTINATION lib)
