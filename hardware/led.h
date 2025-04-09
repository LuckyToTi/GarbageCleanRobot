#ifndef __LED_H_
#define __LED_H_

void LedRgbGpioInit();
void LedRgbWrite(char led_color, unsigned char bit);
unsigned char LedRgbRead(char led_color);

#endif  // __LED_H_
