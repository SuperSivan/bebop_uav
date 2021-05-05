#ifndef __KEYBOARD_H__
#define __KEYBOARD_H__

void init_keyboard();

void close_keyboard();

int kbhit();

char readch(); /* 相关函数声明 */

#endif // __KEYBOARD_H__