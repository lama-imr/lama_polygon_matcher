#ifndef _Utils_h__
#define _Utils_h__

#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>

double getTime(struct rusage one, struct rusage two);
void getTime(struct rusage *t);
void setRandom();

#endif

