/********************************************************************************************
 * NxWidgets/nxwm/src/capplicationwindow.cxx
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX, NxWidgets, nor the names of its contributors
 *    me be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ********************************************************************************************/

/********************************************************************************************
 * Included Files
 ********************************************************************************************/
 
#include <nuttx/config.h>

#include <nuttx/nx/nxglib.h>

#include "nxconfig.hxx"
#include "cwidgetcontrol.hxx"
#include "cgraphicsport.hxx"

#include "nxwmconfig.hxx"
#include "nxwmglyphs.hxx"
#include "capplicationwindow.hxx"

/********************************************************************************************
 * Pre-Processor Definitions
 ********************************************************************************************/

/********************************************************************************************
 * CApplicationWindow Method Implementations
 ********************************************************************************************/

 using namespace NxWM;

/**
 * CApplicationWindow Constructor
 *
 * @param taskbar.  A pointer to the parent task bar instance.
 */

CApplicationWindow::CApplicationWindow(NXWidgets::CNxTkWindow *window)
{
  // Save the window for later use

  m_window          = window;

  // These will be created with the open method is called

  m_toolbar         = (NXWidgets::CNxToolbar        *)0;

  m_minimizeImage   = (NXWidgets::CImage            *)0;
  m_stopImage       = (NXWidgets::CImage            *)0;
  m_windowLabel     = (NXWidgets::CLabel            *)0;

  m_minimizeBitmap  = (NXWidgets::CRlePaletteBitmap *)0;
  m_stopBitmap      = (NXWidgets::CRlePaletteBitmap *)0;

  m_windowFont      = (NXWidgets::CNxFont           *)0;

  // This will be initialized when the registerCallbacks() method is called

  m_callback        = (IApplicationCallback         *)0;
}

/**
 * CApplicationWindow Destructor
 */

CApplicationWindow::~CApplicationWindow(void)
{
  // Free the resources that we are responsible for

  if (m_minimizeImage)
    {
      delete m_minimizeImage;
    }

  if (m_stopImage)
    {
      delete m_stopImage;
    }

  if (m_windowLabel)
    {
      delete m_windowLabel;
    }

  if (m_minimizeBitmap)
    {
      delete m_minimizeBitmap;
    }

  if (m_stopBitmap)
    {
      delete m_stopBitmap;
    }

  if (m_windowFont)
    {
      delete m_windowFont;
    }

  if (m_toolbar)
    {
      delete m_toolbar;
    }

  // We didn't create the window.  That was done by the task bar,
  // But we will handle destruction of with window as a courtesy.

  if (m_window)
    {
      delete m_window;
    }
}

/**
 * Initialize window.  Window initialization is separate from
 * object instantiation so that failures can be reported.
 *
 * @return True if the window was successfully initialized.
 */

bool CApplicationWindow::open(void)
{
  // Open the toolbar

  m_toolbar = m_window->openToolbar(CONFIG_NXWM_TOOLBAR_HEIGHT);
  if (!m_toolbar)
    {
      // We failed to open the toolbar

      return false;
    }

  // Get the width of the display

  struct nxgl_size_s windowSize;
  if (!m_toolbar->getSize(&windowSize))
    {
      return false;
    }

  // Get the CWidgetControl associated with the toolbar

  NXWidgets::CWidgetControl *control = m_toolbar->getWidgetControl();
  if (!control)
    {
      return false;
    }

  // Create STOP bitmap container

  m_stopBitmap = new NXWidgets::CRlePaletteBitmap(&g_stopBitmap);
  if (!m_stopBitmap)
    {
      return false;
    }

  // Create the STOP application icon at the right of the toolbar

  struct nxgl_point_s iconPos;
  struct nxgl_size_s  iconSize;

  // Get the height and width of the stop bitmap

  iconSize.w = m_stopBitmap->getWidth();
  iconSize.h = m_stopBitmap->getHeight();

  // The default CImage has borders enabled with thickness of the border
  // width.  Add twice the thickness of the border to the width and height.
  // (We could let CImage do this for us by calling
  // CImage::getPreferredDimensions())

  iconSize.w += 2 * 1;
  iconSize.h += 2 * 1;

  // Pick an X/Y position such that the image will position at the right of
  // the toolbar and centered vertically.

  iconPos.x = windowSize.w - iconSize.w;

  if (iconSize.h >= windowSize.h)
    {
      iconPos.y = 0;
    }
  else
    {
      iconPos.y = (windowSize.h - iconSize.h) >> 1;
    }

  // Now we have enough information to create the image

  m_stopImage = new NXWidgets::CImage(control, iconPos.x, iconPos.y, iconSize.w,
                                      iconSize.h, m_stopBitmap);
  if (!m_stopImage)
    {
      return false;
    }

  // Configure 'this' to receive mouse click inputs from the image

  m_stopImage->setBorderless(true);
  m_stopImage->addWidgetEventHandler(this);

  // Create MINIMIZE application bitmap container

  m_minimizeBitmap = new NXWidgets::CRlePaletteBitmap(&g_minimizeBitmap);
  if (!m_minimizeBitmap)
    {
      return false;
    }

  // Get the height and width of the stop bitmap

  iconSize.w = m_minimizeBitmap->getWidth();
  iconSize.h = m_minimizeBitmap->getHeight();

  // The default CImage has borders enabled with thickness of the border
  // width.  Add twice the thickness of the border to the width and height.
  // (We could let CImage do this for us by calling
  // CImage::getPreferredDimensions())

  iconSize.w += 2 * 1;
  iconSize.h += 2 * 1;

  // Pick an X/Y position such that the image will position at the right of
  // the toolbar and centered vertically.

  iconPos.x -= iconSize.w;

  if (iconSize.h >= windowSize.h)
    {
      iconPos.y = 0;
    }
  else
    {
      iconPos.y = (windowSize.h - iconSize.h) >> 1;
    }

  // Now we have enough information to create the image

  m_minimizeImage = new NXWidgets::CImage(control, iconPos.x, iconPos.y, iconSize.w,
                                          iconSize.h, m_minimizeBitmap);
  if (!m_minimizeImage)
    {
      return false;
    }

  // Configure 'this' to receive mouse click inputs from the image

  m_minimizeImage->setBorderless(true);
  m_minimizeImage->addWidgetEventHandler(this);

  // The rest of the toolbar will hold the left-justified application label
  // Create the default font instance

  m_windowFont = new NXWidgets::CNxFont(CONFIG_NXWM_DEFAULT_FONTID,
                                        CONFIG_NXWM_DEFAULT_FONTCOLOR,
                                        CONFIG_NXWM_TRANSPARENT_COLOR);
  if (!m_windowFont)
    {
      return false;
    }

  // Get the height and width of the text display area

  iconSize.w = iconPos.x;
  iconSize.h = windowSize.h;

  iconPos.x  = 0;
  iconPos.y  = 0;

  // Now we have enough information to create the label

  m_windowLabel = new NXWidgets::CLabel(control, iconPos.x, iconPos.y,
                                        iconSize.w, iconSize.h, "");
  if (!m_windowLabel)
    {
      return false;
    }

  // Configure the label

  m_windowLabel->setBorderless(true);
  m_windowLabel->setTextAlignmentHoriz(NXWidgets::CLabel::TEXT_ALIGNMENT_HORIZ_LEFT);
  m_windowLabel->setTextAlignmentVert(NXWidgets::CLabel::TEXT_ALIGNMENT_VERT_CENTER);
  m_windowLabel->setRaisesEvents(false);
  return true;
}

/**
 * Re-draw the application window
 */

void CApplicationWindow::redraw(void)
{
  // Get the widget control from the task bar

  NXWidgets::CWidgetControl *control = m_toolbar->getWidgetControl();

  // Get the graphics port for drawing on the background window

  NXWidgets::CGraphicsPort *port = control->getGraphicsPort();

  // Get the size of the window

  struct nxgl_size_s windowSize;
  if (!m_toolbar->getSize(&windowSize))
    {
      return;
    }

  // Fill the entire tool bar with the non-shadowed border color

  port->drawFilledRect(0, 0, windowSize.w, windowSize.h,
                       CONFIG_NXTK_BORDERCOLOR1);

  // Then draw the images

  m_stopImage->enableDrawing();
  m_stopImage->redraw();
  m_stopImage->setRaisesEvents(true);

  m_minimizeImage->enableDrawing();
  m_minimizeImage->redraw();
  m_minimizeImage->setRaisesEvents(true);

  // And draw the window label

  m_windowLabel->enableDrawing();
  m_windowLabel->redraw();
}

/**
 * The application window is hidden (either it is minimized or it is
 * maximized, but not at the top of the hierarchy)
 */

void CApplicationWindow::hide(void)
{
  // Disable the images

  m_stopImage->disableDrawing();
  m_stopImage->setRaisesEvents(false);

  m_minimizeImage->disableDrawing();
  m_minimizeImage->setRaisesEvents(false);

  // Disable the window label

  m_windowLabel->disableDrawing();
}

/**
 * Recover the contained NXTK window instance
 *
 * @return.  The window used by this application
 */

NXWidgets::INxWindow *CApplicationWindow::getWindow(void) const
{
  return static_cast<NXWidgets::INxWindow*>(m_window);
}

/**
 * Set the window label
 *
 * @param appname.  The name of the application to place on the window
 */

void CApplicationWindow::setWindowLabel(NXWidgets::CNxString &appname)
{
  m_windowLabel->setText(appname);
}

/**
 * Register to receive callbacks when toolbar icons are selected
 */

void CApplicationWindow::registerCallbacks(IApplicationCallback *callback)
{
  m_callback = callback;
}

/**
 * Simulate a mouse click on the minimize icon.  This method is only
 * used during automated testing of NxWM.
 */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
void CApplicationWindow::clickMinimizeIcon(int index)
{
  // Get the size and position of the widget

  struct nxgl_size_s imageSize;
  m_minimizeImage->getSize(imageSize);

  struct nxgl_point_s imagePos;
  m_minimizeImage->getPos(imagePos);

  // And click the image at its center

  m_minimizeImage->click(imagePos.x + (imageSize.w >> 1), imagePos.y + (imageSize.h >> 1));
}
#endif

/**
 * Simulate a mouse click on the stop applicaiton icon.  This method is only
 * used during automated testing of NxWM.
 */

#if defined(CONFIG_NXWM_UNITTEST) && !defined(CONFIG_NXWM_TOUCHSCREEN)
void CApplicationWindow::clickStopIcon(int index)
{
  // Get the size and position of the widget

  struct nxgl_size_s imageSize;
  m_stopImage->getSize(imageSize);

  struct nxgl_point_s imagePos;
  m_stopImage->getPos(imagePos);

  // And click the image at its center

  m_stopImage->click(imagePos.x + (imageSize.w >> 1), imagePos.y + (imageSize.h >> 1));
}
#endif

/**
 * Handle a mouse button click event.
 *
 * @param e The event data.
 */

void CApplicationWindow::handleClickEvent(const NXWidgets::CWidgetEventArgs &e)
{
  // Ignore the event if no callback is registered

  if (m_callback)
    {
      // Check the stop application image

      if (m_stopImage->isClicked())
        {
          // Notify the controlling logic that the application should be stopped

          m_callback->close();
        }

      // Check the minimize image (only if the stop application image is not pressed)

      else if (m_minimizeImage->isClicked())
        {
          // Notify the controlling logic that the application should be miminsed

          m_callback->minimize();
        }
    }
}
