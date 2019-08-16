#ifndef  __DEBUG_ASSERT_H__
#define  __DEBUG_ASSERT_H__
 
#ifdef  __cplusplus
#define  DEBUG_ASSERT_BEGIN extern "C" {
#define  DEBUG_ASSERT_END   }
#else
#define  DEBUG_ASSERT_BEGIN
#define  DEBUG_ASSERT_END
#endif
 
DEBUG_ASSERT_BEGIN

#if  defined(DEBUG_ASSERT) &&  DEBUG_ASSERT > 0
#define  DEBUG_ASSERT_NULL(x)                 \
do                                            \
{                                             \
    if ((x) == NULL) {                        \
        while (1);                            \
    }                                         \
} while (0);                                
#else
#define  DEBUG_ASSERT_NULL(x)
#endif

#if  defined(DEBUG_ASSERT) &&  DEBUG_ASSERT > 0
#define  DEBUG_ASSERT_FALSE(x)                \
do                                            \
{                                             \
    if ((x) == 0) {                           \
        while (1);                            \
    }                                         \
} while (0);                                
#else
#define  DEBUG_ASSERT_FALSE(x)
#endif



DEBUG_ASSERT_END
#endif
