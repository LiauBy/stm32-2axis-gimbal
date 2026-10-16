#ifndef __TIMER_H
#define __TIMER_H

void Timer_Init(void);

/* ★ 独立看门狗：超时约 1s，主循环每轮 IWDG_Feed() */
void IWDG_Init(void);
void IWDG_Feed(void);

#endif
