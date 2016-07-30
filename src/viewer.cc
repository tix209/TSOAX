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
 * This file implements the render window of SOAX program.
 */


#include "./viewer.h"
#include <sstream>
#include <QFileInfo>  // NOLINT(build/include_order)
#include <QString>
#include "QVTKWidget.h"
#include "QVTKInteractor.h"
#include "vtkRenderer.h"
#include "vtkImageData.h"
#include "vtkWindowToImageFilter.h"
#include "vtkPNGWriter.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkOrientationMarkerWidget.h"
#include "vtkAxesActor.h"
#include "vtkCaptionActor2D.h"
#include "vtkTextProperty.h"
#include "vtkCornerAnnotation.h"
#include "vtkEventQtSlotConnect.h"
#include "vtkPointPicker.h"

#include "./image_plane.h"
#include "./slice_planes.h"
#include "./volume_rendering.h"
#include "./snake_actor.h"
#include "./junction_actor.h"
#include "./actor_color.h"

namespace soax {

Viewer::Viewer(QVTKWidget *qvtk) : qvtk_(qvtk), renderer_(vtkRenderer::New()),
                                   slot_connector_(vtkEventQtSlotConnect::New()),
                                   picker_(vtkPointPicker::New()),
                                   snapshot_path_("..") {
  renderer_->SetActiveCamera(viewpoint_.camera());
  qvtk_->GetRenderWindow()->AddRenderer(renderer_);
  qvtk_->GetRenderWindow()->SetMultiSamples(8);
  qvtk_->GetInteractor()->SetPicker(picker_);
  picker_->SetTolerance(0.01);
}

Viewer::~Viewer() {
  if (snake_actor_) {
    delete snake_actor_;
  }

  if (comparing_snake_actor_) {
    delete comparing_snake_actor_;
  }

  if (junction_actor_) {
    delete junction_actor_;
  }

  if (comparing_junction_actor_) {
    delete comparing_junction_actor_;
  }

  if (corner_text_) {
    corner_text_->Delete();
  }

  if (orientation_marker_) {
    orientation_marker_->Delete();
  }

  if (volume_) {
    delete volume_;
  }

  if (slice_planes_) {
    delete slice_planes_;
  }

  if (image_plane_) {
    delete image_plane_;
  }

  picker_->Delete();
  slot_connector_->Delete();
  renderer_->Delete();
  // delete qvtk_;
}

void Viewer::ToggleImagePlane(bool state, vtkImageData *image) {
  if (state) {
    if (!image_plane_) {
      SetupImagePlane(image);
    }
    image_plane_->Show(renderer_);
    renderer_->ResetCamera();
  } else {
    image_plane_->Hide(renderer_);
  }
  Render();
}

void Viewer::ToggleSlicePlanes(bool state, vtkImageData *image) {
  if (state) {
    if (!slice_planes_) {
      SetupSlicePlanes(image);
    }
    slice_planes_->Show();
    renderer_->ResetCamera();
  } else {
    slice_planes_->Hide();
  }
  Render();
}

void Viewer::ToggleVolumeRendering(bool state, vtkImageData *image) {
  if (state) {
    if (!volume_) {
      SetupVolumeRendering(image);
    }
    volume_->Show(renderer_);
    renderer_->ResetCamera();
  } else {
    volume_->Hide(renderer_);
  }
  Render();
}

void Viewer::ToggleBoundingBox(bool state, vtkImageData *image) {
  if (state) {
    if (!volume_) {
      SetupVolumeRendering(image);
    }
    volume_->ShowBoundingBox(renderer_);
  } else {
    volume_->HideBoundingBox(renderer_);
  }
  Render();
}

void Viewer::ToggleCornerText(bool state, vtkImageData *image) {
  if (state) {
    if (!corner_text_) {
      UpdateCornerTextImageInfo(image);
    }
    renderer_->AddViewProp(corner_text_);
  } else {
    renderer_->RemoveViewProp(corner_text_);
  }
  Render();
}

void Viewer::RemoveSnakes() {
  RemoveSnakeActor(snake_actor_);
}

void Viewer::SetupSnakes(const SnakeContainer &snakes) {
  SetupSnakeActor(&snake_actor_, snakes);
}

void Viewer::RemoveComparingSnakes() {
  RemoveSnakeActor(comparing_snake_actor_);
}

void Viewer::SetupComparingSnakes(const SnakeContainer &snakes) {
  SetupSnakeActor(&comparing_snake_actor_, snakes);
}

void Viewer::RemoveJunctions() {
  RemovePointActor(junction_actor_);
}

void Viewer::SetupJunctions(const PointContainer &points) {
  SetupPointActor(&junction_actor_, points);
}

void Viewer::RemoveComparingJunctions() {
  RemovePointActor(comparing_junction_actor_);
}

void Viewer::SetupComparingJunctions(const PointContainer &points) {
  SetupPointActor(&comparing_junction_actor_, points);
}

/**
 * Precondition: image_plane_ or slice_planes_ are already shown in the
 * renderer_.
 */
void Viewer::UpdateImagePlane(vtkImageData *image) {
  if (slice_planes_)
    slice_planes_->Hide();

  if (volume_)
    volume_->Hide(renderer_);

  SetupImagePlane(image);
  image_plane_->Show(renderer_);
  Render();
}

/**
 * Precondition: image_plane_ or slice_planes_ are already shown in the
 * renderer_.
 */
void Viewer::UpdateSlicePlanes(vtkImageData *image) {
  if (image_plane_)
    image_plane_->Hide(renderer_);

  SetupSlicePlanes(image);
  slice_planes_->Show();
  Render();
}

void Viewer::UpdateVolumeRendering(vtkImageData *image) {
  volume_->Hide(renderer_);
  SetupVolumeRendering(image);
  volume_->Show(renderer_);
  Render();
}

void Viewer::UpdateCornerTextImageInfo(vtkImageData *image) {
  if (!corner_text_) {
    corner_text_ = vtkCornerAnnotation::New();
  }

  std::ostringstream buffer;
  double range[2];
  image->GetScalarRange(range);
  buffer << "Intensity: [" << range[0] << ", " << range[1] << "]";
  corner_text_->SetText(2, buffer.str().c_str());
  Render();
}

void Viewer::UpdateCornerTextSnake(const QString &filename) {
  std::string text;
  if (!corner_text_->GetText(3)) {
    text = filename.toStdString() + " (Magenta)\n";
  } else {
    text = corner_text_->GetText(3);
    size_t newline_pos = text.find('\n');
    if (newline_pos == 0) {
      text.insert(0, filename.toStdString() + " (Magenta)");
    } else {
      text.replace(0, text.find(" (Magenta)"), filename.toStdString());
    }
  }
  corner_text_->SetText(3, text.c_str());
}

void Viewer::UpdateCornerTextComparingSnake(const QString &filename) {
  std::string text = "\n";
  if (!corner_text_->GetText(3)) {
    text += filename.toStdString() + " (Yellow)";
  } else {
    text = corner_text_->GetText(3);
    size_t newline_pos = text.find('\n');
    if (newline_pos == text.size() - 1) {
      text += filename.toStdString() + " (Yellow)";
    } else {
      size_t start = newline_pos + 1;
      size_t end = text.find(" (Yellow)");
      text.replace(start, end - start, filename.toStdString());
    }
  }
  corner_text_->SetText(3, text.c_str());
}

void Viewer::SaveWindowImage(const QString &filename) {
  vtkWindowToImageFilter *filter = vtkWindowToImageFilter::New();
  filter->SetInput(qvtk_->GetRenderWindow());
  filter->Update();
  vtkPNGWriter *writer = vtkPNGWriter::New();
  writer->SetInputConnection(filter->GetOutputPort());
  writer->SetFileName(filename.toStdString().c_str());
  writer->Write();
  filter->Delete();
  writer->Delete();
  snapshot_path_ = QFileInfo(filename).absolutePath();
}

QString Viewer::GetViewpointPath() const {
  return viewpoint_.path();
}

void Viewer::LoadViewpoint(const QString &filename) {
  viewpoint_.LoadFromFile(filename);
  Render();
}

void Viewer::SaveViewpoint(const QString &filename) {
  viewpoint_.SaveToFile(filename);
}

void Viewer::SaveViewpoint() {
  viewpoint_.SaveDefault();
}

void Viewer::Reset() {
  if (image_plane_) {
    delete image_plane_;
    image_plane_ = nullptr;
  }

  if (slice_planes_) {
    delete slice_planes_;
    slice_planes_ = nullptr;
  }

  if (volume_) {
    delete volume_;
    volume_ = nullptr;
  }

  if (orientation_marker_) {
    orientation_marker_->Delete();
    orientation_marker_ = nullptr;
  }

  if (corner_text_) {
    corner_text_->Delete();
    corner_text_ = nullptr;
  }

  if (snake_actor_) {
    delete snake_actor_;
    snake_actor_ = nullptr;
  }

  if (comparing_snake_actor_) {
    delete comparing_snake_actor_;
    comparing_snake_actor_ = nullptr;
  }

  if (junction_actor_) {
    delete junction_actor_;
    junction_actor_ = nullptr;
  }

  if (comparing_junction_actor_) {
    delete comparing_junction_actor_;
    comparing_junction_actor_ = nullptr;
  }
}

/********************** Private Slots **********************/

void Viewer::ToggleNormal(bool state) {
  if (state) {
    slot_connector_->Connect(qvtk_->GetInteractor(),
                             vtkCommand::LeftButtonPressEvent,
                             this, SLOT(SelectSnakeForView()));
    slot_connector_->Connect(qvtk_->GetInteractor(),
                             vtkCommand::RightButtonPressEvent,
                             this, SLOT(DeselectSnakeForView()));
  } else {
    slot_connector_->Disconnect(qvtk_->GetInteractor(),
                                vtkCommand::LeftButtonPressEvent,
                                this, SLOT(SelectSnakeForView()));
    slot_connector_->Disconnect(qvtk_->GetInteractor(),
                                vtkCommand::RightButtonPressEvent,
                                this, SLOT(DeselectSnakeForView()));
  }
}

void Viewer::SelectSnakeForView() {
  picker_->Pick(qvtk_->GetInteractor()->GetEventPosition()[0],
                qvtk_->GetInteractor()->GetEventPosition()[1],
                0, renderer_);
  vtkActor *actor = picker_->GetActor();
  if (actor) {
    if (snake_actor_ && snake_actor_->Has(actor))
      snake_actor_->SelectForView(actor);
    else if (comparing_snake_actor_ && comparing_snake_actor_->Has(actor))
      comparing_snake_actor_->SelectForView(actor);
  }
}

void Viewer::DeselectSnakeForView() {
  picker_->Pick(qvtk_->GetInteractor()->GetEventPosition()[0],
                qvtk_->GetInteractor()->GetEventPosition()[1],
                0, renderer_);
  vtkActor *actor = picker_->GetActor();
  if (actor) {
    if (snake_actor_ && snake_actor_->Has(actor))
      snake_actor_->DeselectForView(actor);
    else if (comparing_snake_actor_ && comparing_snake_actor_->Has(actor))
      comparing_snake_actor_->DeselectForView(actor);
  }
}

// void Viewer::HighlightCorrespondingSnake() {
//   snake_actor_->HighlightCorrespondingSnake();
// }

void Viewer::ToggleSnakes(bool state) {
  ToggleSnakeActor(state, snake_actor_);
}

void Viewer::ToggleComparingSnakes(bool state) {
  ToggleSnakeActor(state, comparing_snake_actor_);
}

void Viewer::ResetViewpoint() {
  viewpoint_.LoadDefault();
  renderer_->ResetCamera();
  Render();
}

void Viewer::ToggleOrientationMarker(bool state) {
  if (!orientation_marker_)
    SetupOrientationMarker();
  orientation_marker_->SetEnabled(state);
  if (state)
    orientation_marker_->InteractiveOff();
  Render();
}

void Viewer::ToggleJunctions(bool state) {
  TogglePointActor(state, junction_actor_);
}

void Viewer::ToggleComparingJunctions(bool state) {
  TogglePointActor(state, comparing_junction_actor_);
}


void Viewer::ToggleClipSnakes(bool state) {
  if (state) {
  } else {
  }
}

void Viewer::ColorByAzimuthalAngle(bool state) {
  if (state) {
  } else {
  }
}

void Viewer::ColorByPolarAngle(bool state) {
  if (state) {
  } else {
  }
}

/********************** Private Methods **********************/
void Viewer::Render() {
  qvtk_->GetRenderWindow()->Render();
}

void Viewer::SetupImagePlane(vtkImageData *image) {
  if (!image_plane_) {
    image_plane_ = new ImagePlane(image);
  } else {
    image_plane_->Update(image);
  }
}

void Viewer::SetupSlicePlanes(vtkImageData *image) {
  if (!slice_planes_) {
    slice_planes_ = new SlicePlanes(image, qvtk_->GetInteractor());
  } else {
    slice_planes_->Update(image);
  }
}

void Viewer::SetupVolumeRendering(vtkImageData *image) {
  if (!volume_) {
    volume_ = new VolumeRendering(image);
  } else {
    volume_->Update(image);
  }
}

void Viewer::SetupOrientationMarker() {
  orientation_marker_ = vtkOrientationMarkerWidget::New();
  vtkAxesActor *axes = vtkAxesActor::New();
  const int font_size = 14;
  axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(font_size);
  axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(font_size);
  axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->
      SetFontSize(font_size);
  orientation_marker_->SetOrientationMarker(axes);
  orientation_marker_->SetInteractor(qvtk_->GetInteractor());
  orientation_marker_->SetViewport(0, 0, 0.3, 0.3);
  axes->Delete();
}

void Viewer::ToggleSnakeActor(bool state, SnakeActor *actor) {
  if (!actor) return;
  if (state) {
    actor->Show(renderer_);
  } else {
    actor->Hide(renderer_);
  }
  Render();
}

void Viewer::RemoveSnakeActor(SnakeActor *actor) {
  if (actor) {
    actor->Hide(renderer_);
    actor->Reset();
  }
  Render();
}

void Viewer::SetupSnakeActor(SnakeActor **actor,
                             const SnakeContainer &snakes) {
  if (!*actor) {
    if (actor == &comparing_snake_actor_) {
      *actor = new SnakeActor(snakes, ActorColor::kYellow, 8.0, 0.5);
    } else {
      *actor = new SnakeActor(snakes, ActorColor::kMagenta, 3.0, 1.0);
    }
  } else {
    (*actor)->Update(snakes);
  }
  (*actor)->Show(renderer_);
}

void Viewer::TogglePointActor(bool state, JunctionActor *actor) {
  if (!actor) return;
  if (state) {
    actor->Show(renderer_);
  } else {
    actor->Hide(renderer_);
  }
  Render();
}

void Viewer::RemovePointActor(JunctionActor *actor) {
  if (actor) {
    actor->Hide(renderer_);
    actor->Reset();
  }
  Render();
}

void Viewer::SetupPointActor(JunctionActor **actor,
                             const PointContainer &points) {
  double *color = ActorColor::kGreen;
  double radius = 1.0;
  double opacity = 1.0;
  if (actor == &comparing_junction_actor_) {
    color = ActorColor::kWhite;
    radius = 2.0;
    opacity = 0.5;
  }

  if (!*actor) {
    *actor = new JunctionActor(points, color, radius, opacity);
  } else {
    (*actor)->Update(points, color, radius, opacity);
  }
  (*actor)->Show(renderer_);
}


}  // namespace soax
