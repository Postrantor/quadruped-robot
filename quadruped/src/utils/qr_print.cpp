/**
 * @author Robot Motion and Vision Laboratory at East China Normal University
 * @brief
 * @date 2022
 * @copyright MIT License
 */

#include "utils/qr_print.hpp"

void printf_color(PrintColor color, const char *fmt, ...) {
  auto color_id = (uint32_t)color;
  if (color_id) printf("\033[1;%dm", (uint32_t)color + 30);
  va_list args;
  va_start(args, fmt);
  vprintf(fmt, args);
  va_end(args);
  printf("\033[0m");
}

void fprintf_color(PrintColor color, FILE *stream, const char *fmt, ...) {
  auto color_id = (uint32_t)color;
  if (color_id) fprintf(stream, "\033[1;%dm", (uint32_t)color + 30);
  va_list args;
  va_start(args, fmt);
  vfprintf(stream, fmt, args);
  va_end(args);
  fprintf(stream, "\033[0m");
}
