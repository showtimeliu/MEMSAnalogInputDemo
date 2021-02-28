#ifndef MTI_SWITCHOS_H
#define MTI_SWITCHOS_H

#include "MTIDefinitions.h"

#ifdef MTI_WINDOWS
	#include <conio.h>
	#define CLEARSCREEN "cls"
#endif

#ifdef MTI_UNIX
	#include <termios.h>
	#include <sys/select.h>
	#include <sys/ioctl.h>
	#include <unistd.h>
	#include <stdio.h>
	#define CLEARSCREEN "clear"

/* Define _getch() and _kbhit() for Linux since they are Win-only functions */
static int _getch() 
{
	struct termios oldt, newt;
	int            ch;
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	ch = getchar();
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	return ch;
}

static int _kbhit() {
	static const int STDIN = 0;
	static bool initialized = false;

	if (!initialized) {
	    // Use termios to turn off line buffering
		termios term;
		tcgetattr(STDIN, &term);
		term.c_lflag &= ~ICANON;
		tcsetattr(STDIN, TCSANOW, &term);
		setbuf(stdin, NULL);
		initialized = true;
	}

	int bytesWaiting;
	ioctl(STDIN, FIONREAD, &bytesWaiting);
	return bytesWaiting;
}

#endif

#endif