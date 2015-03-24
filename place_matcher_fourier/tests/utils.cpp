
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/resource.h>
#include <time.h>

#include <pm_fourier/utils.h>

void setRandom(){

  time_t t;

  time(&t);
  srand(t);

}

void getTime(struct rusage *t)
{
  getrusage(RUSAGE_SELF,t);
}

/**
 * vraci rozdil casu mezi two a one (two-one). vraci v sec
 */
double getTime(struct rusage one, struct rusage two)
{
  const unsigned long as = one.ru_utime.tv_sec;
  const unsigned long bs = two.ru_utime.tv_sec;
  const unsigned long aus = one.ru_utime.tv_usec;
  const unsigned long bus = two.ru_utime.tv_usec;

  return (double)((double)bs-(double)as) + 
    (double)((double)bus-(double)aus) / 1000000.0;
}


