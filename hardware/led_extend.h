#ifndef __LED_EXTEND_H_
#define __LED_EXTEND_H_

void LedExtendRgbGpioInit();
void LedExtendRgbWrite(char led_color, unsigned char bit);
unsigned char LedExtendRgbRead(char led_color);

#endif  // __LED_H_
