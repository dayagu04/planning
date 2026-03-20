
#include "viz_text.h"
#include "viz2d_geometry.h"

namespace planning {

int viz_func_state_machine(const FuncStateMachine::FuncStateMachine &state,
                           viz2d_image *viz2d, const Pose2D *veh_pose) {
  CvPoint text_center;

  text_center.x = 100;
  text_center.y = 100;

  int text_y_step = 20;

  std::string text;

  if (state.has_current_state()) {
    text = FuncStateMachine::FunctionalState_Name(state.current_state());
  } else {
    text = "No state";
  }
  text_center.y += text_y_step;
  viz_draw_text_relative_to_cv_coordinate(viz2d, text, viz2d_colors_white,
                                          text_center);

  return 0;
}

int viz2d_draw_chassis_feedback(viz2d_image *viz2d, const double v,
                                const double wheel, const uint32_t gear) {
  CvPoint point1, point2;

  CvScalar text_color;

  viz2d_color text_color_index = viz2d_colors_white;

  if (viz2d == nullptr) {
    return -1;
  }

  viz2d_get_color(&text_color, text_color_index);

  CvPoint origin, center, text_center;
  CvPoint y_end_left_tail, y_end_right_tail;
  int columns, rows, origin_row_index, origin_column_index;
  IplImage *img;
  CvFont windows_font;

  columns = viz2d->columns;
  rows = viz2d->rows;
  origin_row_index = viz2d->origin_row_index;
  origin_column_index = viz2d->origin_column_index;
  img = viz2d->image;
  origin.x = origin_column_index;
  origin.y = origin_row_index;

  viz2d_get_cvfont(&windows_font, &(viz2d->font));

  center.x = 100;
  center.y = 20;

  // text
  text_center = center;

  std::string gear_str;
  switch (gear) {
    case 0:
      gear_str = "P";
    case 1:
      gear_str = "R";
    case 2:
      gear_str = "N";
    case 3:
      gear_str = "D";
      break;
    default:
      gear_str = "P";
      break;
  }

  std::string string = "chassis info: speed:";
  string = string + std::to_string(v) + " m/s" +
           " steer:" + std::to_string(wheel) + ", " + gear_str;

  cvPutText(viz2d->image, string.c_str(), text_center, &windows_font,
            text_color);

  // fill box
  CvPoint box_center;
  box_center.x = columns / 2;
  box_center.y = rows - 80;

  viz_draw_box_in_cv_frame(viz2d, &box_center, 150, 800,
                           viz2d_colors_black_gray, true);

  // speed value

  text_center.x = columns / 2 - 250;
  text_center.y = rows - 80;

  double v_kmh = v * 3.6;

  char str[128];
  sprintf(str, " %.2f", v_kmh);
  windows_font.hscale = 1.5;
  windows_font.vscale = 1.5;
  windows_font.thickness = 2.0;
  cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

  // speed unit
  text_center.x = columns / 2 - 220;
  text_center.y += 50;
  sprintf(str, " km/h");
  windows_font.hscale = 1.0;
  windows_font.vscale = 1.0;
  windows_font.thickness = 1.0;
  cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

  // wheel

  if (wheel >= 0.0) {
    sprintf(str, "+%.2f", wheel);
  } else {
    sprintf(str, "-%.2f", std::fabs(wheel));
  }

  text_center.x += 160;
  text_center.y = rows - 80;

  windows_font.hscale = 1.5;
  windows_font.vscale = 1.5;
  windows_font.thickness = 2.0;
  cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

  // speed unit
  text_center.y += 50;
  sprintf(str, " degree");
  windows_font.hscale = 1.0;
  windows_font.vscale = 1.0;
  windows_font.thickness = 1.0;
  cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

  return 0;
}

int viz2d_draw_control_commond_info(viz2d_image *viz2d, const double acc,
                                    const double wheel,
                                    Common::GearCommandValue control_gear) {
  CvPoint point1, point2;

  CvScalar text_color;

  viz2d_color text_color_index = viz2d_colors_white;

  if (viz2d == nullptr) {
    return -1;
  }

  viz2d_get_color(&text_color, text_color_index);

  CvPoint origin, center, text_center;
  CvPoint y_end_left_tail, y_end_right_tail;
  int columns, rows, origin_row_index, origin_column_index;
  IplImage *img;
  CvFont windows_font;

  columns = viz2d->columns;
  rows = viz2d->rows;
  origin_row_index = viz2d->origin_row_index;
  origin_column_index = viz2d->origin_column_index;
  img = viz2d->image;
  origin.x = origin_column_index;
  origin.y = origin_row_index;

  viz2d_get_cvfont(&windows_font, &(viz2d->font));

  center.x = 100;
  center.y = 40;

  // text
  text_center = center;
  std::string string = "commond info: acc:";

  string = string + std::to_string(acc) +
           " m/ss, steer:" + std::to_string(wheel) + ", " +
           Common::GearCommandValue_Name(control_gear);

  cvPutText(viz2d->image, string.c_str(), text_center, &windows_font,
            text_color);

  // acc
  char str[128];
  if (acc >= 0.0) {
    sprintf(str, "+%.2f", acc);
  } else {
    sprintf(str, "-%.2f", std::fabs(acc));
  }

  text_center.x = columns / 2 + 170;
  text_center.y = rows - 80;

  windows_font.hscale = 1.5;
  windows_font.vscale = 1.5;
  windows_font.thickness = 2.0;
  cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

  text_center.y += 50;
  sprintf(str, " m/s^2");
  windows_font.hscale = 1.0;
  windows_font.vscale = 1.0;
  windows_font.thickness = 1.0;
  cvPutText(viz2d->image, str, text_center, &windows_font, text_color);

  return 0;
}

}  // namespace planning