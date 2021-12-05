#include <stdio.h>
#include "Time.h"
#include "i2c-lcd.h"

void Time_count(void)
{
	uint8_t Hour;
	uint8_t Minute;
	uint8_t Second;
	uint8_t Charbuff[20];
	
	lcd_send_cmd(0x80|0x54);
	sprintf((char*)Charbuff,"Time: %d",Second);
	lcd_send_string((char*)Charbuff);
	Second++;
	if(Second == 60)
	{
		Second = 0;
		Minute++;
		
		if(Minute == 60)
		{
			Minute = 0;
			Hour++;
			
			if(Hour == 24)
			{
				Hour = 0;
			}
			else
			{
				//Do nothing
			}
		}
		else
		{
			//Do nothing
		}
	}
	else
	{
		//Do nothing
	}
		
}