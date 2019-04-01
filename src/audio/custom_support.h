/*
 * custom_support.h
 *
 *  Created on: 29 mars 2019
 *      Author: nlantz
 */

#ifndef SRC_AUDIO_CUSTOM_SUPPORT_H_
#define SRC_AUDIO_CUSTOM_SUPPORT_H_

#include <stddef.h>

#define __inhibit_loop_to_libcall  __attribute__ ((__optimize__ ("-fno-tree-loop-distribute-patterns")))

void *
__inhibit_loop_to_libcall
opus_memmove (void *dst_void,
	const void *src_void,
	size_t length);

void *
opus_mempcpy (void *dst0,
	const void *src0,
	size_t len0);

void *
__inhibit_loop_to_libcall
opus_memset (void *m,
	int c,
	size_t n);

#define OVERRIDE_OPUS_MOVE
#define OPUS_MOVE(dst, src, n) (opus_memmove((dst), (src), (n)*sizeof(*(dst)) + 0*((dst)-(src)) ))

#define OVERRIDE_OPUS_COPY
#define OPUS_COPY(dst, src, n) (opus_mempcpy((dst), (src), (n)*sizeof(*(dst)) + 0*((dst)-(src)) ))

#define OVERRIDE_OPUS_CLEAR
#define OPUS_CLEAR(dst, n) (opus_memset((dst), 0, (n)*sizeof(*(dst))))



#endif /* SRC_AUDIO_CUSTOM_SUPPORT_H_ */
