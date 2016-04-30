/* VT100 terminal command */
#define ESC	(0x1B)
#define TERM_CLEAR_SCREEN		printf("%c[2J", ESC)
#define TERM_CURSOR_HOME		printf("%c[H", ESC)
#define TERM_TEXT_RED			printf("%c[31m", ESC)
#define TERM_TEXT_GREEN			printf("%c[32m", ESC)
#define TERM_TEXT_WHITE			printf("%c[37m", ESC)
#define TERM_BKGRD_WHITE		printf("%c[47m", ESC)
#define TERM_TEXT_BLUE			printf("%c[34m", ESC)
#define TERM_TEXT_BLACK			printf("%c[30m", ESC)
#define TERM_TEXT_DEFAULT		printf("%c[0m", ESC)
#define TERM_CURSOR_POS(v,h)	printf("%c[%d;%dH", ESC,v,h)
//
#define TERM_CURSOR_SAVE		printf("%c7", ESC)
#define TERM_CURSOR_RESTORE		printf("%c8", ESC)

void Term_Banner( void);
