#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include "keyboard.h"

#define key_ESC 27

static struct termios initial_settings, new_settings;

static int peek_character = -1;         /* 用于测试一个按键是否被按下 */

/* 检测键盘按键的函数 */

int kbhit()

{
    char ch;

    int nread;

    if ( peek_character != -1 )

        return(1);

    new_settings.c_cc[VMIN] = 0;

    tcsetattr( 0, TCSANOW, &new_settings );

    nread = read( 0, &ch, 1 );

    new_settings.c_cc[VMIN] = 1;

    tcsetattr( 0, TCSANOW, &new_settings );

    if ( nread == 1 )
    {
        peek_character = ch;

        return(1);
    }

    return(0);
}

/* 用来接收按下的按键，并peek_character = -1恢复状态 */

char readch()

{
    char ch;

    if ( peek_character != -1 )
    {
        ch = peek_character;

        peek_character = -1;

        return(ch);
    }

    read( 0, &ch, 1 );

    return(ch);
}

/* 配置终端函数 */

void init_keyboard()

{
    tcgetattr( 0, &initial_settings );

    new_settings = initial_settings;

    new_settings.c_lflag &= ~ICANON;

    new_settings.c_lflag &= ~ECHO;

    new_settings.c_lflag &= ~ISIG;

    new_settings.c_cc[VMIN] = 1;

    new_settings.c_cc[VTIME] = 0;

    tcsetattr( 0, TCSANOW, &new_settings );
}


void close_keyboard()

{
    tcsetattr( 0, TCSANOW, &initial_settings );
}

// int main(int argc, char const *argv[])
// {
//     char ch;

//     int flag = 0;

//     long int key_cnt = 0;

//     init_keyboard();

//     printf( "You can put ESC to quit!\n" );
//     while ( ch != 27 )
//     {
//         if ( kbhit() )
//         {
//             key_cnt = 0;

//             ch = readch();

//             if ( ch != 27 )

//                 printf( "You put %c ! Only put ESC can quit! \n", ch );
//         }
//         else
//         {
//             key_cnt++;
//         }

//         if(key_cnt > 5)
//         {
//             printf( "ha \n" );
//         }

//         usleep(10000);
//     }

//     close_keyboard();

//     return 0;
// }