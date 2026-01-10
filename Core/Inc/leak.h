#ifndef __LEAK_H__
#define __LEAK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef struct {
    uint16_t value;    
} Leak_t;

extern Leak_t leak;

void LEAK_DumpState(Leak_t* s, char* buff, int len);
void LEAK_Init(Leak_t *l);
uint16_t LEAK_Get(Leak_t *l);

#ifdef __cplusplus
}
#endif

#endif /* __LEAK_H__ */
