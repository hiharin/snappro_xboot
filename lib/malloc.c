/*
 * Copyright © 2011 Logic Product Development, Inc.
 * <peter.barada@logicpd.com>
 *
 * This file is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 or (at your option) any
 * later version.
 *
 * This file is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this file; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <common.h>
#include <asm/string.h>

#if 0
#define MALLOC_DEBUG(args...) printf(args)
#else
#define MALLOC_DEBUG(args...)
#endif

void *malloc_base;
void *malloc_end;
void *malloc(size_t size)
{
	void *p;

	/* Round size up to a long to keep malloc_base aligned */
	size = ((size + (sizeof(long) - 1)) & ~(sizeof(long) - 1));

	/* If we march off the end return a NULL */
	if (malloc_base + size > malloc_end)
		return NULL;
	p = malloc_base;
	malloc_base += size;
	MALLOC_DEBUG("%s: malloc(%d) = %p\n", __FUNCTION__, size, p);
	return p;
}

void *calloc(size_t nemb, size_t size)
{
	void *p;

	size *= nemb;
	p = malloc(size);
	if (p)
		memset(p, 0, size);
	return p;
}

void free(void *ptr)
{
	/* Null function, can't return memory */
}

void malloc_init(void *base, size_t size)
{
	malloc_base = base;
	malloc_end = malloc_base + size;
	MALLOC_DEBUG("%s: base %p size %#x\n", __FUNCTION__, malloc_base, size);
}
