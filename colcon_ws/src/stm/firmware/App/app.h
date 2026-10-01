/*
 * app.h - main.c 에서 부르는 함수 두 개
 *
 *   app_init() : main.c 의 USER CODE BEGIN 2 에서 한 번
 *   app_loop() : main.c 의 while(1) 안 USER CODE BEGIN 3 에서 계속
 */
#ifndef APP_H
#define APP_H

void app_init(void);
void app_loop(void);

#endif /* APP_H */
