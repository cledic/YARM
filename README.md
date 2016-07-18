# YARM
YARM development using "**Atmel Studio 7**" and "**AFS**" Library.

Library to setup the [YARM](http://www.acmesystems.it/yarm) board from [ACME systems](http://www.acmesystems.it)

## To use this libraries follow this simple guide line.

1. You must download and install **Atmel Studio 7** from here: http://www.atmel.com/Microsite/atmel-studio/
2. Launch "Atmel Studio 7" then select "**File -> New -> Project**". Inside the "**New Project**" window click on and select "**GCC C ASF Board Project**" 
3. Type in the name of your project, and click "**OK**".
4. A new windows appear. Now you must choose the MCU. Select inside "**Device Family**" pop-up the family "**SAML21**"; then select the "**ATSAML21E18B**" device and click OK.
5. The new project will be created. Now is time to insert the "**Atmel Software Framework (ASF)**" modules.
6. Select "**Project -> ASF Wizard**" from the menu, after a while, on the left side of the window will appear a list of the available modules.
   With the help of the "Search for modules" text box, search and then "Add >>" the following modules:
   * Generic board support (driver)
   * Delay routines (service) [*cycle*] ^1
   * EXTINT - External Interrupt (driver) [*callback*]
   * PORT - GPIO Pin Control (driver)
   * RTC -  Real Time Counter Driver (driver) [*count_callback*]
   * SERCOM I2C - Master Mode I2C (driver) [*polled*]
   * SERCOM SPI - Serial Peripheral Interface (driver) [*callback*]
   * SERCOM USART - Serial Communication (driver) [*polled*]
   * SYSTEMS - Core Systems Driver (driver)
   * Standard serial I/O (stdio) (driver)
   * *N.B. Some of this modules can be configured using a pop-up menu on the right of the name. The correct value to set is
   inside the square bracket of each modules.*
7. Click on "**Apply**". At this point your project is ready to use this libraries.
8. Inside the project window, right click on the "**src**" folder inside the "**Solution Explorer**" window on the right. Choose "**Add -> Existing Item**" to copy the source library inside your project.
9. Remenber to add inside in "**Properties -> Toolchain -> ARM/GNU Linker -> Miscellaneous**" the option: "**-lc -u _printf_float**" to visualize the float value with "*printf*".

(1) On Atmel Studio 7.0.634 and ASF 3.31.0 set the Delay routines (service) [*systick*]

See the PDF file inside the "**doc**" folder for a step by step guide by screenshot.
