#ifndef WS2811_H_
#define WS2811_H_

// For 4 MHz SPI + DMA

#define WS2811_LEDS 32	//32

typedef struct ws2811_color {
	uint8_t red, green, blue;
} ws2811_color;

void WS2811_Init(SPI_HandleTypeDef * spi_handler);
void WS2811_SetDiodeColor(int16_t diode_id, uint32_t color);
void WS2811_SetDiodeColorStruct(int16_t diode_id, ws2811_color color);
void WS2811_SetDiodeRGB(int16_t diode_id, uint8_t R, uint8_t G, uint8_t B);
void WS2811_SetDiodeHSV(int16_t diode_id, uint16_t Hue, uint8_t Saturation, uint8_t Brightness);
uint32_t WS2811_GetColor(int16_t diode_id);
uint8_t* WS2811_GetPixels(void);
void WS2811_Refresh();

// color correction
uint8_t sine8(uint8_t x);
uint8_t gamma8(uint8_t x);
#endif /* WS2811_H_ */
