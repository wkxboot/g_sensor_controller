#ifndef  __COMM_UTILS_H
#define  __COMM_UTILS_H


#define  ASSERT_NULL_POINTER(X)                \
{                                              \
 if((X) == NULL)                               \
 return -1;                                    \
}                                       



#ifndef   IS_POWER_OF_TWO
#define   IS_POWER_OF_TWO(A)   (((A) != 0) && ((((A) - 1) & (A)) == 0))
#endif

#ifndef   MIN
#define   MIN(A,B)             ((A) > (B) ? (B) :(A))
#endif

#ifndef   MAX
#define   MAX(A,B)             ((A) > (B) ? (A) :(B))
#endif



#endif

