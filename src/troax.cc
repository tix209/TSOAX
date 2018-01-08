/**
 * Copyright (C) 2017 Lehigh University.
 *
 * This program is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 *
 * Author: Ting Xu (xuting.bme@gmail.com)
 *
 * This file is the main function of the SOAX program.
 */


#include <QApplication>
#include "include/main_window.h"
#include "QVTKOpenGLWidget.h"
#include "vtkGenericOpenGLRenderWindow.h"

int main(int argc, char **argv) {
#ifndef __APPLE__
  vtkOpenGLRenderWindow::SetGlobalMaximumNumberOfMultiSamples(0);
#endif

  QSurfaceFormat::setDefaultFormat(QVTKOpenGLWidget::defaultFormat());
  QApplication app(argc, argv);

#ifdef __APPLE__
  app.setAttribute(Qt::AA_UseHighDpiPixmaps);
  // app.setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
#endif

  soax::MainWindow window;
  const int width = 800;
  const int height = 600;
  window.resize(width, height);
  window.show();

  return app.exec();
}
