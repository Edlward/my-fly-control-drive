#ifdef RT_USING_FINSH
#include <finsh.h>
static rt_uint8_t led_inited = 0;


#define LED2_PIN 4
void rt_hw_led_init()
{
	rt_pin_mode(LED2_PIN, PIN_MODE_OUTPUT);
}

void led(rt_uint32_t led, rt_uint32_t value)
{
     /* init led configuration if it's not inited. */
    if (!led_inited)
     {
         rt_hw_led_init();
         led_inited = 1;
     }
    if ( led == 0 )
     {
          /* set led status */
         switch (value)
          {
         case 0:
              rt_hw_led_off(0);
              break;
         case 1:
              rt_hw_led_on(0);
              break;
         default:
              break;
          }
     }
    if ( led == 1 )
     {
          /* set led status */
         switch (value)
          {
         case 0:
              rt_hw_led_off(1);
              break;
         case 1:
              rt_hw_led_on(1);
              break;
         default:
              break;
          }
     }
}
FINSH_FUNCTION_EXPORT(led, set led[0 - 1] on[1] or off[0].)
#endif

