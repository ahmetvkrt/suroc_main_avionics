#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

void main_avionic_init(void);
void main_avionic_loop(void);

extern float current_alt_m;

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
