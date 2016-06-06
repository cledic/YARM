/** Example about how to use the Daisy24 library
 *  http://www.acmesystems.it/DAISY-24
 * 
 */
#include "Daisy24_lib.h"

char r1[25];
char r2[25];
uint32_t loop;
uint32_t pb;


int main( void)
{
    // Init the library: configure the I2C channel and initialize the LCD
	Daisy24_LCD_Init();
	
    // Just display the "hello world"...
	Daisy24_LCD_WriteString("Ciao Mondo!");
	delay_ms(2000);
	
    // Loop, 
	while(1)
	{
        // Prepare two string
		//           1234567890123456
		sprintf(r1, "Sens. Salone ");
		sprintf(r2, "T: 23C P:1002hP H:56 RSSI:-50dBm ");
        
        // Configure the Monitor function passing the string and the mode
        // mode 3 == first string fixed and second string looping
		Daisy24_LCD_MonitorInit( r1, r2, 3);
	
        // In a real program you must put this code inside a timer callback
        // to update the string rotation.
		loop=0;
		while(loop<3) {     // in this case I stop the visualization after three time rotation
            // MonitorUpdate update the visualization and return the time the string is looping
			Daisy24_LCD_MonitorUpdate( &loop);
            // PB_Status return the status of the push button
			Daisy24_PB_Status( &pb);
			delay_ms(250);
		}

        // Prepare two string to update the visualization
		//           1234567890123456
		sprintf(r1, "Sens. Letto 1 ");
		sprintf(r2, "T: 22C P:1002hP H:56 RSSI:-43dBm ");
		Daisy24_LCD_MonitorInit( r1, r2, 3);
	
		loop=0;
		while(loop<3) {
			Daisy24_LCD_MonitorUpdate( &loop);
			Daisy24_PB_Status( &pb);
			delay_ms(250);
		}
	}
}
