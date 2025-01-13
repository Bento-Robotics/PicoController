//#ifndef __eduart_drive_h
//#define __eduart_drive_h


#include "micro_rosso.h"
class
EduArt_Drive
{
public:
  EduArt_Drive();

  static bool setup(const char* topics_namespace = "/bento",
                    timer_descriptor &timer = micro_rosso::timer_report);
};

//#endif //__eduart_drive_h
