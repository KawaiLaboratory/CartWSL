#ifndef _ke_HPP_
#define _ke_HPP_

#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//kbhit関数が押されるたびに、入力出力設定をいちいち変えてるため、美しくない。
//プログラムが終わる際に入力を元に戻すようにプログラムを書く場合、途中でプログラムが強制終了しても設定が元に戻るようにしなければいけないため、このままのプログラムのほうが最適かもしれない。

int kbhit()
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

void key_event_string (char *command){ //64B
	int c;
	if (kbhit()){
		while((c=getchar())!=-1 && c!=0) {
			*command = c;
			++command;
		}
  }
}

int key_event_char (){
	if (kbhit()) {
		return getchar();
	}else{
		return -1;
	}
}

#endif