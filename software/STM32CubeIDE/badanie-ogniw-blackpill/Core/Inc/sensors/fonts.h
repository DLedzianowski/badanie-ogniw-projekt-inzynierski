#ifndef __FONTS_H__
#define __FONTS_H__

#include <stdint.h>

#define FONT1	Arial_Narrow8x12
#define FONT1w	8
#define FONT1h	12
#define FONT2	Arial_Narrow10x13
#define FONT2w	10
#define FONT2h	13
#define FONT3	Arial_Narrow12x16
#define FONT3w	12
#define FONT3h	16
#define FONT4	Arial_Narrow15x19
#define FONT4w	15
#define FONT4h	19

typedef struct {
    const uint8_t width;
    const uint8_t height;
    const uint16_t *data;
} FontDef;

extern FontDef img_bar;
extern FontDef img_thunder;
extern FontDef img_charging;
extern FontDef img_batery;
extern FontDef img_arrowup;
extern FontDef img_arrowdown;

extern const uint8_t Arial_Narrow8x12[];
extern const uint8_t Arial_Narrow10x13[];
extern const uint8_t Arial_Narrow12x16[];
extern const uint8_t Arial_Narrow15x19[];

#endif // __FONTS_H__
