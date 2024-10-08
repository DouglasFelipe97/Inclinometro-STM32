#include "OLED_Icons.h"

/* Icon in XBM */

const uint8_t icon_clock[] = {
		13, 13,
		0xE0, 0x00, 0xB8, 0x03, 0x0C, 0x06, 0x06, 0x0C, 0x02, 0x08, 0x03, 0x18,
		0xC1, 0x17, 0x43, 0x18, 0x42, 0x08, 0x46, 0x0C, 0x0C, 0x06, 0xB8, 0x03,
		0xE0, 0x00,
};

const uint8_t icon_engrenagem[] = {
		13, 13,
		  0xE0, 0x00, 0xFC, 0x07, 0xFE, 0x0F, 0xFE, 0x0F, 0x1E, 0x0F, 0x0F, 0x1E,
		  0x0F, 0x1E, 0x0F, 0x1E, 0x1E, 0x0F, 0xFE, 0x0F, 0xFE, 0x0F, 0xFC, 0x07,
		  0xE0, 0x00,
};

const uint8_t icon_bateria[] = {
		18, 13,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xDD, 0xDD, 0x00,
		0xDD, 0xDD, 0x03, 0xDD, 0xDD, 0x03, 0xDD, 0xDD, 0x03, 0xDD, 0xDD, 0x03,
		0xDD, 0xDD, 0x03, 0xDD, 0xDD, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00,
};

const uint8_t logo[] = {
		64, 64,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
		0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x03, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0,
		0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x0E, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x70, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38,
		0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x3C, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x1C, 0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E,
		0x7E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0xE7, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x87, 0xC3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x83,
		0x83, 0x01, 0x00, 0x00, 0x00, 0x00, 0x80, 0x81, 0x81, 0x03, 0x00, 0x00,
		0x00, 0x00, 0xC0, 0x01, 0x80, 0x07, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x0C,
		0xC0, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x60, 0x1C, 0xC0, 0x0E, 0x00, 0x00,
		0x00, 0x00, 0x70, 0x9E, 0xE1, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x38, 0xCE,
		0x71, 0x38, 0x00, 0x00, 0x00, 0x00, 0x18, 0xC7, 0x31, 0x30, 0x00, 0x00,
		0x00, 0x00, 0x1C, 0xE3, 0x39, 0x70, 0x00, 0x00, 0x00, 0x00, 0x8E, 0xE3,
		0x9C, 0xE1, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x73, 0x8E, 0xE3, 0x00, 0x00,
		0x00, 0x00, 0x0E, 0x3F, 0xCE, 0x73, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x9E,
		0xC7, 0x31, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x9C, 0xE3, 0x38, 0x00, 0x00,
		0x00, 0x00, 0x38, 0x8E, 0x73, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x70, 0x8E,
		0x33, 0x0C, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x07, 0x38, 0x0E, 0x00, 0x00,
		0x00, 0x00, 0xE0, 0x0F, 0x1C, 0x07, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x1F,
		0x8C, 0x03, 0x00, 0x00, 0x00, 0x00, 0x80, 0x39, 0x8E, 0x03, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x38, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x03, 0x00, 0x00, 0x70,
		0x00, 0x00, 0x00, 0xC0, 0x01, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0xC0,
		0x00, 0x00, 0x00, 0xB0, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x50,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x38, 0x8C, 0xF9, 0xE3, 0xF1, 0xC3, 0x70, 0x06,
		0x38, 0x8C, 0xF9, 0xF3, 0xF3, 0xE3, 0x71, 0x06, 0x38, 0x8C, 0xE1, 0xB8,
		0x33, 0xE3, 0xF1, 0x06, 0x7C, 0x8C, 0x61, 0x18, 0x33, 0xE3, 0xF1, 0x06,
		0x7C, 0x8C, 0x61, 0x18, 0x37, 0xE3, 0xF1, 0x07, 0x7C, 0x8C, 0x61, 0x18,
		0xF7, 0xF3, 0xF3, 0x07, 0x6C, 0x8C, 0x61, 0x18, 0xF7, 0xF3, 0xF3, 0x07,
		0xEE, 0x8C, 0x61, 0x18, 0x37, 0x37, 0xF3, 0x07, 0xFE, 0x8C, 0x61, 0x18,
		0x33, 0xF7, 0xB3, 0x07, 0xFE, 0xDC, 0x61, 0xB8, 0xF3, 0xFF, 0x37, 0x07,
		0xF6, 0xF8, 0x61, 0xF0, 0xF3, 0xBB, 0x37, 0x07, 0xC7, 0xF8, 0x60, 0xF0,
		0xF1, 0x1B, 0x37, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFB, 0x02, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xCC, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x07,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

const uint8_t logov2[] = {
		128, 64,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xC0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x0F, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
		0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x78, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x78, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E,
		0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x0F, 0xF8, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x07, 0xFC, 0x03, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x03,
		0x9E, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xE0, 0x81, 0x0F, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xC0, 0x07, 0x1E, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0x80,
		0x07, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x3C, 0x80, 0x03, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0x00, 0xFC, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x8F, 0x07,
		0x00, 0xFE, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x80, 0x87, 0x87, 0x00, 0xCF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC3, 0xC7, 0x83, 0x87, 0x07, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xC1, 0xC3,
		0xC3, 0x03, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xF0, 0xE0, 0xE1, 0xE3, 0x01, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xF8, 0xF0, 0xF1, 0x00, 0x3C, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x78, 0x78,
		0x78, 0x1C, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x3C, 0xF0, 0x3C, 0x3C, 0x3E, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x78, 0xE0, 0x1F, 0x1E, 0x3E, 0x3C, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xC0, 0x8F,
		0x0F, 0x1F, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xE0, 0x81, 0xC7, 0x87, 0x07, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xC3, 0xC3, 0xC7, 0x83, 0x07, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE7, 0x81,
		0xE3, 0xC1, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xFF, 0x01, 0xF0, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0x78, 0xF8, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC, 0x07,
		0x3C, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x18, 0x0F, 0x1E, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x0F, 0x38, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBC,
		0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xF8, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00,
		0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x80, 0x1E, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x80, 0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0,
		0xE0, 0xF0, 0xFF, 0xF1, 0xC3, 0x7F, 0xE0, 0xE0, 0xE0, 0x06, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xF0, 0xE0, 0xF0, 0xFF, 0xF9, 0xCF, 0xFF, 0xF0, 0xE1,
		0xE1, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0xE1, 0x70, 0x3C, 0x3C,
		0xCF, 0xE3, 0xF0, 0xE1, 0xE3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8,
		0xE1, 0x70, 0x38, 0x1E, 0xDC, 0xE1, 0xF0, 0xE3, 0xE7, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x98, 0xE3, 0x70, 0x38, 0x0E, 0xDC, 0x7F, 0xB8, 0xE3,
		0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x9C, 0xE3, 0x70, 0x38, 0x0E,
		0xDC, 0x7F, 0x38, 0xE7, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
		0xE3, 0x70, 0x38, 0x0E, 0xDC, 0xF3, 0xBC, 0xE7, 0xFC, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xFE, 0xE7, 0x78, 0x38, 0x1E, 0xDE, 0xE1, 0xFD, 0xEF,
		0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xE7, 0x3D, 0x38, 0x7C,
		0xCF, 0xE3, 0xFF, 0xEF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
		0xCE, 0x3F, 0x38, 0xF8, 0xC7, 0xFF, 0x0E, 0xEE, 0xF0, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x07, 0x8E, 0x1F, 0x38, 0xF0, 0xC3, 0x7F, 0x0E, 0xFC,
		0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFC,
		0xE1, 0x63, 0x30, 0x78, 0x30, 0x80, 0x07, 0x78, 0x18, 0x1C, 0x00, 0x00,
		0x00, 0x80, 0xFF, 0xFF, 0xF1, 0xE7, 0x38, 0xFE, 0x71, 0xE0, 0x1F, 0xFE,
		0x19, 0x3C, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE, 0x79, 0xE7, 0x39, 0xFF,
		0x33, 0xF0, 0x3D, 0xDF, 0x19, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x18, 0x0E,
		0x1C, 0xE0, 0x3B, 0x87, 0x33, 0x70, 0x38, 0x07, 0x18, 0x3E, 0x00, 0x00,
		0x00, 0x00, 0x18, 0xFE, 0x1C, 0xE0, 0xB3, 0x03, 0x37, 0x70, 0xF0, 0x03,
		0x18, 0x77, 0x00, 0x00, 0x00, 0x00, 0x18, 0xFE, 0x1D, 0xE0, 0xBF, 0x03,
		0x37, 0x30, 0xF0, 0x83, 0x19, 0x67, 0x00, 0x00, 0x00, 0x00, 0x18, 0x8E,
		0x1C, 0x60, 0xBE, 0x03, 0x73, 0x70, 0xF0, 0x83, 0x19, 0xF7, 0x00, 0x00,
		0x00, 0x00, 0x18, 0x0E, 0x1C, 0x60, 0x3E, 0x87, 0x33, 0x70, 0x38, 0x87,
		0x99, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x18, 0xFE, 0xF9, 0x67, 0x3C, 0xFF,
		0xF3, 0xE7, 0x3F, 0xFF, 0x99, 0xFB, 0x01, 0x00, 0x00, 0x00, 0x1C, 0xFE,
		0xF1, 0x67, 0x38, 0xFE, 0xF1, 0xE7, 0x1F, 0xFE, 0xD9, 0xC1, 0x01, 0x00,
		0x00, 0x00, 0x18, 0xFC, 0xE1, 0x61, 0x30, 0x78, 0xF0, 0x87, 0x07, 0x78,
		0x98, 0x80, 0x01, 0x00, };


const uint8_t maxforte[] = {
		128, 64,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F,
		0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0x1F, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x7F, 0xF8, 0x03, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF,
		0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x80, 0xFF, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xE7, 0x0F, 0xF8, 0x07, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xE1,
		0x83, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xE0, 0x7F, 0xFE, 0x3F, 0xFE, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x9F, 0x7F, 0xFF, 0xFC, 0x07, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xCF, 0x3F,
		0xFE, 0xF9, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x80, 0xE7, 0x1F, 0xFC, 0xF3, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF3, 0x4F, 0xF8, 0xE7, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFB, 0x67,
		0xF0, 0xEF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xF8, 0xF9, 0x73, 0xE0, 0xCF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xF9, 0x7B, 0xC0, 0xDF, 0x07, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xF9, 0x7D,
		0xC0, 0xDF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xFE, 0xFD, 0x7C, 0x80, 0xDF, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xF9, 0x7C, 0x80, 0xDF, 0x3F, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF9, 0x7C,
		0x80, 0xCF, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xC0, 0xFB, 0x7D, 0x80, 0xCF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF3, 0x79, 0xC0, 0xEF, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0xF7, 0x63,
		0xE0, 0xE7, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xC0, 0xEF, 0x0F, 0xF0, 0xF3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0x9F, 0xFF, 0xFF, 0xF9, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x3F, 0xFF,
		0x7F, 0xFE, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xE0, 0xFF, 0xF8, 0x0F, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF1, 0x03, 0xE0, 0xFF, 0x01, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xFF,
		0xFF, 0xC3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x0F,
		0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0xC0, 0x07, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0xFE, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xC0, 0x07, 0xFC, 0x03, 0x1E, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x0F, 0xFC, 0x07, 0x0F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80,
		0x1F, 0xF8, 0x8F, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x1F, 0xF0, 0xC1, 0x0F, 0x1F, 0xFC, 0x8F, 0xE7, 0xFF, 0xF8, 0x1F, 0xFE,
		0xDF, 0xFF, 0xFF, 0x3F, 0x3F, 0xF0, 0xC1, 0x0F, 0x3E, 0xEE, 0xDF, 0xE3,
		0x03, 0x3E, 0x3C, 0x3E, 0x7E, 0xFC, 0xF0, 0x21, 0x3F, 0xF8, 0xC1, 0x1F,
		0x7C, 0xE7, 0xFF, 0xE1, 0x01, 0x1F, 0xF8, 0x3E, 0x7C, 0x78, 0xF0, 0x01,
		0x7F, 0xFC, 0x61, 0x1F, 0xFC, 0xC3, 0xFF, 0xE0, 0x01, 0x0F, 0xF8, 0x3E,
		0xF8, 0x7C, 0xF0, 0x01, 0xFF, 0xFC, 0x61, 0x3E, 0xF8, 0x81, 0xFF, 0xE0,
		0x81, 0x0F, 0xF0, 0x3F, 0x78, 0x7C, 0xF0, 0x01, 0xFF, 0xFE, 0x71, 0x3E,
		0xF0, 0x01, 0xFF, 0xE0, 0xBF, 0x0F, 0xF0, 0x3F, 0x7C, 0x7C, 0xF0, 0x1F,
		0xF7, 0xF7, 0x31, 0x7C, 0xF0, 0x03, 0xFF, 0xE1, 0xBF, 0x0F, 0xF0, 0xBF,
		0x1F, 0x7C, 0xF0, 0x0F, 0xE7, 0xF7, 0xF9, 0x7F, 0xF0, 0x03, 0xFF, 0xE3,
		0x81, 0x0F, 0xF0, 0xBF, 0x0F, 0x7C, 0xF0, 0x01, 0xE7, 0xF3, 0xF9, 0xFF,
		0xF8, 0x07, 0xFF, 0xE3, 0x81, 0x0F, 0xF8, 0x3E, 0x1F, 0x7C, 0xF0, 0x01,
		0xC7, 0xF3, 0x1D, 0xF8, 0x9C, 0x8F, 0xFF, 0xE7, 0x01, 0x1F, 0xF8, 0x3E,
		0x1F, 0x7C, 0xF0, 0x01, 0xC7, 0xF1, 0x0D, 0xF0, 0x8F, 0xDF, 0xFB, 0xEF,
		0x01, 0x3E, 0x7C, 0x3E, 0x3E, 0x7C, 0xF0, 0x01, 0x86, 0xF0, 0x0F, 0xF0,
		0x07, 0xFF, 0xF1, 0xFF, 0x01, 0xFC, 0x3F, 0x3E, 0x7C, 0x7C, 0xF0, 0x7F,
		0x04, 0xC0, 0x07, 0xC0, 0x03, 0xFC, 0xE0, 0xFF, 0x01, 0xE0, 0x07, 0x3C,
		0x70, 0x78, 0xE0, 0x1F, 0x00, 0x00, 0x00, 0x80, 0x01, 0xF8, 0xC0, 0x3F,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x78, 0xC0, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x3C, 0x80, 0xFF, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1E, 0x00, 0xFF,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFE, 0x03, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC,
		0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0xF8, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, };


const uint8_t icon_chave[] = {
		13, 13,
		0x80, 0x03, 0xC0, 0x01, 0xC0, 0x01, 0xC0, 0x19, 0xC0, 0x1F, 0xC0, 0x1F,
		0xF0, 0x0F, 0x78, 0x00, 0x7C, 0x00, 0x3E, 0x00, 0x1F, 0x00, 0x0F, 0x00,
		0x06, 0x00,
};
