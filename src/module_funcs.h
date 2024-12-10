#ifndef _MODULE_FUNCS_H_
#define _MODULE_FUNCS_H_

void update_state(void *arg, long period);
void update_feedback(void *arg, long period);

#ifdef CONNECTION_GPIO
void gpio_communicate(void *arg, long period);
#endif

#endif
