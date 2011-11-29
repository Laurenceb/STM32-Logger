

#define delay()						\
do {							\
  register unsigned int i;				\
  for (i = 0; i < 10000000; ++i)			\
    __asm__ __volatile__ ("nop\n\t":::"memory");	\
} while (0)
