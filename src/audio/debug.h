
#ifndef __DEBUG_H__
#define __DEBUG_H__

#include <misc/__assert.h>

#define ASSERT(expr)          __ASSERT_NO_MSG(expr)

#define APP_ERROR_CHECK(x) \
	do {           \
		int ret = (x == NRFX_SUCCESS)?1:0;\
		__ASSERT_NO_MSG(ret); \
	} while (false)

#define APP_ERROR_CHECK_BOOL(x) ASSERT(x)


#endif // __DEBUG_H__
