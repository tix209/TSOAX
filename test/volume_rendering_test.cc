/**
 * Copyright (C) 2015 Lehigh University.
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
 * This file test the VolumeRendering class.
 */

#include "../volume_rendering.h"
#include "gtest/gtest.h"
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include "../image_reader.h"


namespace soax {

class VolumeRenderingTest : public ::testing::Test {
 protected:
  VolumeRenderingTest() {}

  void Visualize() {
    vtkRenderWindow * ren_win = vtkRenderWindow::New();
    vtkRenderer * ren = vtkRenderer::New();
    ren_win->AddRenderer(ren);
    vtkRenderWindowInteractor *iren = vtkRenderWindowInteractor::New();
    iren->SetRenderWindow(ren_win);
    vr_->Show(ren);
    vr_->ShowBoundingBox(ren);
    ren->ResetCamera();
    ren_win->Render();
    iren->Start();

    ren->Delete();
    ren_win->Delete();
    iren->Delete();
  }

  VolumeRendering *vr_ = nullptr;
  ImageReader reader;
};

TEST_F(VolumeRenderingTest, Constructor) {
  QString path = "../../rel4.0/test/data/fz.mha";
  reader.ReadFile(path);
  vr_ = new VolumeRendering(reader.GetImage());
  // Visualize();
  delete vr_;
}

TEST_F(VolumeRenderingTest, Update) {
  QString path = "../../rel4.0/test/data/seq_iso_4-1.mha";
  reader.set_nslices_per_frame(74);
  reader.ReadFile(path);
  vr_ = new VolumeRendering(reader.GetImage());

  vr_->Update(reader.GetImage(1));
  // Visualize();
  delete vr_;
}

TEST_F(VolumeRenderingTest, UpdateDisplayRange) {
  QString path = "../../rel4.0/test/data/fz.mha";
  reader.ReadFile(path);
  vr_ = new VolumeRendering(reader.GetImage());
  vr_->SetDisplayRange(100, 200);

  // Visualize();
  delete vr_;
}


} // namespace soax
