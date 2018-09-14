
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

IF (MSVC_VERSION STREQUAL 1900)
  ADD_LIBRARY(RcsGui ${LIBRARY_LINKING_TYPE} ${RCS_GUI_SRCS} )
  TARGET_LINK_LIBRARIES(RcsGui RcsCore Qt5::Core Qt5::Gui Qt5::Widgets ${QWT_LIBRARIES})
  TARGET_INCLUDE_DIRECTORIES(RcsGui PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${Qt5Core_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${QWT_INCLUDE_DIRS})
  SET_TARGET_PROPERTIES(RcsGui PROPERTIES AUTOMOC TRUE)
ELSE()
SET(RCS_Qt4_RES )
QT4_ADD_RESOURCES(RCS_RES_H ${RCS_Qt4_RES})
QT4_WRAP_UI(RCS_UI_CPP ${RCS_Qt4_UI})
QT4_WRAP_CPP(RCS_MOC_CPP ${RCS_Qt4_SRC})
ADD_LIBRARY(RcsGui ${LIBRARY_LINKING_TYPE} ${RCS_GUI_SRCS} ${RCS_MOC_CPP} ${RCS_UI_CPP} ${RCS_RES_H})
TARGET_INCLUDE_DIRECTORIES(RcsGui PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${QT_QTGUI_INCLUDE_DIR}
  ${QT_QTCORE_INCLUDE_DIR}
    ${QT_INCLUDE_DIR})
TARGET_INCLUDE_DIRECTORIES(RcsGui PRIVATE ${QWT_INCLUDE_DIRS})
TARGET_LINK_LIBRARIES(RcsGui RcsCore ${QWT_LIBRARIES} Qt4::QtGui)
ENDIF()

# Mark for export
RCS_ADD_EXPORT_LIBRARY(RcsGui)
