


#include <viseffect/visEffect.h>
// GPIO clock peripheral enable command
#define WS2812B_GPIO_CLK_ENABLE() __HAL_RCC_GPIOC_CLK_ENABLE()
// LED output port
#define WS2812B_PORT GPIOC
// LED output pins
#define WS2812B_PINS (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3)
// How many LEDs are in the series
#define WS2812B_NUMBER_OF_LEDS 60
// Number of paralel LED strips on the SAME gpio. Each has its own buffer.


// RGB Framebuffers
uint8_t frameBuffer[3*60];
uint8_t frameBuffer2[3*20];




int main (void)
{
visInit();




}