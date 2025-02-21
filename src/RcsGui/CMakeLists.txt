
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
TextEditGui.cpp
STTGui.cpp
PPSGui.cpp
PPSWidget.cpp
Rcs_guiFactory.c
SimpleSlider.cpp
Slider1Dof.cpp
SliderCheck1Dof.cpp
TaskWidget.cpp
DblSlider.cpp
DynamicDataPlot.cpp
ExampleGui.cpp
CmdLineWidget.cpp
AsyncWidget.cpp
AsyncGuiFactory.cpp 
${RCS_HIGHGUI_SRCS}
)

IF(WIN32)
  SET_SOURCE_FILES_PROPERTIES( ${RCS_GUI_SRCS} PROPERTIES LANGUAGE CXX )
  
  #IF(CREATE_WIN_DLL)
    ADD_DEFINITIONS(-DRCSGUI_EXPORTS)
  #ENDIF(CREATE_WIN_DLL)

ENDIF()

FIND_PACKAGE(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

IF (APPLE)
  set(CMAKE_FIND_FRAMEWORK ONLY)
  find_library(QWT
    NAMES qwt
    HINTS /opt/homebrew/opt/qwt-qt5/lib/
    REQUIRED)
  if (QWT)
    message("QWT found: ${QWT}")
    include_directories(${QWT}/Headers)
    link_libraries(${QWT})
    SET (QWT_LIBRARIES ${QWT}/qwt)
    SET (QWT_INCLUDE_DIRS ${QWT}/Headers)
    message("QWT_INCLUDE_DIRS: ${QWT_INCLUDE_DIRS}")
    message("QWT_LIBRARIES: ${QWT_LIBRARIES}")
  endif()
ELSE()
  FIND_PACKAGE(Qwt REQUIRED)
ENDIF()

ADD_LIBRARY(RcsGui ${RCS_GUI_SRCS} )
TARGET_LINK_LIBRARIES(RcsGui RcsCore Qt5::Core Qt5::Gui Qt5::Widgets ${QWT_LIBRARIES})
SET_TARGET_PROPERTIES(RcsGui PROPERTIES AUTOMOC TRUE)

TARGET_INCLUDE_DIRECTORIES(RcsGui PUBLIC
    # Header source location
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    
    # Header install location
    $<INSTALL_INTERFACE:include/RcsGui>
    
    # Regular dependencies
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS})
TARGET_INCLUDE_DIRECTORIES(RcsGui PRIVATE ${QWT_INCLUDE_DIRS})

# Mark for export
RCS_ADD_EXPORT_LIBRARY(RcsGui)

# Install the headers
INSTALL(
  DIRECTORY . 
  DESTINATION include/RcsGui
  FILES_MATCHING PATTERN "*.h*"
)

# Install the binaries
INSTALL(TARGETS RcsGui EXPORT RcsExport
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib/Rcs
        )

IF(WIN32)   # Manually install Qt runtime DLLs
  GET_TARGET_PROPERTY(QT5_CORE_DLL Qt5::Core LOCATION)
  GET_TARGET_PROPERTY(QT5_GUI_DLL Qt5::Gui LOCATION)
  GET_TARGET_PROPERTY(QT5_WIDGETS_DLL Qt5::Widgets LOCATION)
  INSTALL(FILES ${QT5_CORE_DLL} ${QT5_GUI_DLL} ${QT5_WIDGETS_DLL} DESTINATION bin)
ENDIF()
